import rospy

class TestableComponentsRegistry:
    @classmethod
    def register(cls, name, component_info):
        rospy.set_param(f"/components/{name}", component_info)

    @classmethod
    def get(cls, name):
        return rospy.get_param(f"/components/{name}", None)
