#!/usr/bin/env python3
import rospy
import rosgraph
import rosservice
import json
import roslib.message
import roslib.srvs
import yaml

class ROSJsonGenerator:
    def __init__(self):
        rospy.init_node('ros_json_generator', anonymous=False)
        self.master = rosgraph.Master('/ros_json_generator')
        try:
            state = self.master.getSystemState()
        except Exception as e:
            rospy.logerr("Failed to get ROS system state: %s", e)
            state = ([], [], [])
        self.pub_topics = {topic: nodes for topic, nodes in state[0]}
        self.sub_topics = {topic: nodes for topic, nodes in state[1]}
        self.srv_nodes   = {service: nodes for service, nodes in state[2]}

    def serialize_ros_message(self, msg):
        """
        Recursively converts a ROS message (or list/dict of messages) into JSON-serializable types.
        """
        if hasattr(msg, '__slots__'):
            out = {}
            for slot in msg.__slots__:
                try:
                    value = getattr(msg, slot)
                except Exception:
                    value = None
                out[slot] = self.serialize_ros_message(value)
            return out
        elif isinstance(msg, list):
            return [self.serialize_ros_message(item) for item in msg]
        elif isinstance(msg, dict):
            return {k: self.serialize_ros_message(v) for k, v in msg.items()}
        else:
            try:
                json.dumps(msg)
                return msg
            except Exception:
                return str(msg)

    def should_include(self, name):
        """
        Returns False if the topic or service name contains any unwanted keywords.
        """
        exclude_keywords = ["rosout", "rosapi", "roscpp", "get_loggers", "set_logger_level"]
        for keyword in exclude_keywords:
            if keyword in name:
                return False
        return True

    def get_topics(self):
        topics = {}
        published_topics = rospy.get_published_topics()
        for topic, msg_type in published_topics:
            if not self.should_include(topic):
                continue
            sample = roslib.message.get_message_class(msg_type)()
            topics[topic] = {
                "name": topic,
                "type": "topic",
                "messageType": msg_type,
                "sample": self.serialize_ros_message(sample)
            }
        return topics
    
    def call_service_and_get_sample(self, service, service_type):
        """
        Calls the given service using a default-constructed request.
        Returns a dictionary with serialized request and response.

        - If the request has no parameters, the response is returned as is.
        - If the request has one parameter, it iterates through the candidate values:
          ["cameras", "2025-03-06", "(255,0,0)"] until one yields a valid response.
        - If the request has two parameters, the first is set to "cameras" and the
          second is set to a formatted configuration dictionary.
        """
        try:
            srv_class = roslib.message.get_service_class(service_type)
            if srv_class is None:
                return "NaN"
            # Construct a default request
            req = srv_class._request_class()
            # Get the list of fields in the request message
            fields = req.__slots__
            # Wait for the service to become available
            rospy.wait_for_service(service, timeout=5)
            proxy = rospy.ServiceProxy(service, srv_class)

            # No parameters: use the default-constructed request
            if len(fields) == 0:
                response = proxy(req)

            # One parameter: try each candidate until one call returns successfully
            elif len(fields) == 1:
                candidates = ["cameras", "(255,0,0)"]
                response = None
                if service == "/getLogsService":
                    setattr(req, fields[0], "2025-03-06")
                    response = proxy(req)
                elif service == "/getLayoutService":
                    setattr(req, fields[0], "controller")
                    response = proxy(req) 
                else:
                    for candidate in candidates:
                        setattr(req, fields[0], candidate)
                        try:
                            response = proxy(req)
                            # Assuming that a non-None response means success
                            if response is not None:
                                break
                        except Exception as e:
                            rospy.logwarn("Candidate '%s' failed: %s", candidate, e)
                            continue
                if response is None:
                    return "NaN"

            # Two parameters: first is "cameras", second is the formatted configuration
            elif len(fields) == 2:
                setattr(req, fields[0], "cameras")
                config = {
                    "camera1": {
                        "index": "/home/amansour/Zpice/src/gui/src/guiContent/test.mp4",
                        "width": 640,
                        "height": 480,
                        "fps": 20,
                        "port": 8081
                    },
                    "camera2": {
                        "index": "/dev/right_camera",
                        "width": 640,
                        "height": 480,
                        "fps": 30,
                        "port": 8082
                    },
                    "camera3": {
                        "index": "/dev/rapoo_camera",
                        "width": 1280,
                        "height": 960,
                        "fps": 20,
                        "port": 8083
                    }
                }
                config_str = yaml.dump(config)
                setattr(req, fields[1], config_str)
                response = proxy(req)

            else:
                rospy.logwarn("Unhandled number of request parameters: %d", len(fields))
                return "NaN"

            sample = {
                "request": self.serialize_ros_message(req),
                "response": self.serialize_ros_message(response)
            }
            return sample

        except Exception as e:
            rospy.logwarn("Failed to call service %s of type %s: %s", service, service_type, e)
            return "NaN"


    def get_services(self):
        services = {}
        service_list = rosservice.get_service_list()
        for service in service_list:
            if not self.should_include(service):
                continue
            try:
                service_type = rosservice.get_service_type(service)
                args = rosservice.get_service_args(service)
                srv_class = roslib.message.get_service_class(service_type)()
                request = srv_class._request_class()
                response = srv_class._response_class()
                sample = self.call_service_and_get_sample(service, service_type)
            except Exception as e:
                rospy.logwarn("Could not get service type for %s: %s", service, e)
                service_type = "Unknown"
            services[service] = {
                "name": service,
                "type": "service",
                "serviceType": service_type,
                "args": {
                    "request": self.serialize_ros_message(request),
                    "response": self.serialize_ros_message(response)
                },
                "sample": sample
            }
        return services

    def get_actions(self):
        # Actions are built on topics; filter out any unwanted ones.
        action_suffixes = ["/goal", "/cancel", "/status", "/result", "/feedback"]
        topics = rospy.get_published_topics()
        action_groups = {}
        for topic, msg_type in topics:
            if not self.should_include(topic):
                continue
            for suffix in action_suffixes:
                if topic.endswith(suffix):
                    action_name = topic[:-len(suffix)]
                    if action_name not in action_groups:
                        action_groups[action_name] = {}
                    key = suffix.strip("/")
                    sample = roslib.message.get_message_class(msg_type)()
                    action_groups[action_name][key] = {
                        "topic": topic,
                        "messageType": msg_type,
                        "sample": self.serialize_ros_message(sample)
                    }
        actions = {}
        for action_name, components in action_groups.items():
            sample = roslib.message.get_message_class(msg_type)()
            actions[action_name] = {
                "name": action_name,
                "type": "action",
                "actionType": "Derived from components",
                "sample": self.serialize_ros_message(sample),
                "components": list(components.keys())
            }
        return actions

    def generate_json(self):
        data = {}
        topics = self.get_topics()
        services = self.get_services()
        actions = self.get_actions()
        data.update(topics)
        data.update(services)
        data.update(actions)
        return data

    def write_json_file(self, filename="communication_mappings.json"):
        data = self.generate_json()
        try:
            with open(filename, "w") as f:
                json.dump(data, f, indent=4)
            rospy.loginfo("JSON file written to %s", filename)
        except Exception as e:
            rospy.logerr("Failed to write JSON file: %s", e)

if __name__ == "__main__":
    try:
        generator = ROSJsonGenerator()
        rospy.sleep(1.0)  # allow time to collect ROS master information
        generator.write_json_file()
    except rospy.ROSInterruptException:
        pass
