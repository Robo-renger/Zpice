# How to make an API using ROS services

### Prerequisites

find `catkin_install_python` and replace it with :
``` 
catkin_install_python(PROGRAMS ${PYTHON_SCRIPTS}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
```
This will allow that each Server to be launchable from the any launch file.
#### Note, make `chmod +x` for eac server

### Folder structure
```
└── 📁API
    └── 📁servers
        └── __init__.py
        └── ConfigServer.py
    └── 📁services
        └── __init__.py
        └── ConfigService.py
```
You will need an `init.py` in `services` so that you can include it the `servers` and use each service in each corresponding `server`

### Service structure
The service structure will be probably the same for each service as they almost all of them will do CRUD operation
#### Note: If you don't write in OOP please leave the readme IMMEDIATELY
```
required srv imports from ROS

class xxxService

def Getxxx()

def Setxxx()
```
You can also add `Deletexxx()` but a Set will mostly do the job