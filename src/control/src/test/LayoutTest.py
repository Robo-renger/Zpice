#!/usr/bin/env python3

from utils.LayoutManager import LayoutManager

class LayoutTest:
    def __init__(self):
        self.__layoutManager = LayoutManager()

    def testGetLayoutsNames(self):
        print("Testing getLayoutsNames")
        print(self.__layoutManager.getLayoutsNames())

    def testFetchLayout(self):
        print("Testing fetchLayout")
        print(self.__layoutManager.fetchLayout(LayoutManager.CAMERAS))
        print(self.__layoutManager.fetchLayout(LayoutManager.CONTROLLER)) 

    def testSetLayout(self):
        print("Testing setLayout")
        new_layout = {
        "view1": {
            "camera1": {
                "url": "http://example.com/cam1_updated",
                "geometry": {"top": 10, "left": 0, "width": 500, "height": 60}
            },
            "camera2": {
                "url": "http://example.com/cam2_updated",
                "geometry": {"top": 220, "left": 0, "width": 300, "height": 200}
            },
            "camera3": {
                "url": "http://example.com/cam3_updated",
                "geometry": {"top": 10, "left": 220, "width": 200, "height": 300}
            },
            "camera4": {
                "url": "http://example.com/cam4_updated",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera5": {
                "url": "http://example.com/cam5_updated",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera6": {
                "url": "http://example.com/cam6_updated",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            }
        },
        "view2": {
            "camera1": {
                "url": "http://example.com/cam1_view2",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera2": {
                "url": "http://example.com/cam2_view2",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera3": {
                "url": "http://example.com/cam3_view2",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera4": {
                "url": "http://example.com/cam4_view2",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera5": {
                "url": "http://example.com/cam5_view2",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera6": {
                "url": "http://example.com/cam6_view2",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            }
        },
        "view3": {
            "camera1": {
                "url": "http://example.com/cam1_view3",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera2": {
                "url": "http://example.com/cam2_view3",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera3": {
                "url": "http://example.com/cam3_view3",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera4": {
                "url": "http://example.com/cam4_view3",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera5": {
                "url": "http://example.com/cam5_view3",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            },
            "camera6": {
                "url": "http://example.com/cam6_view3",
                "geometry": {"top": 10, "left": 0, "width": 200, "height": 200}
            }
        }
    }
        self.__layoutManager.setLayout(LayoutManager.CAMERAS, new_layout)
        print(self.__layoutManager.fetchLayout(LayoutManager.CAMERAS))

if __name__ == "__main__":
    try:
        layoutTest = LayoutTest()
        # layoutTest.testGetLayoutsNames()
        # layoutTest.testFetchLayout()
        layoutTest.testSetLayout()
    except Exception as e:
        print(f"Error: {e}")