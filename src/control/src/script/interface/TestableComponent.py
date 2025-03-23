#!/usr/bin/env python3
from zope.interface import Interface

class TestableComponent(Interface):
    """
    An interface defining testable components for an underwater system.
    This includes observability and automation tests.
    """

    def observableTest(self) -> None:
        """
        Tests the component using a predefined set of instructions to ensure component works
        as expected. The test should be observed by the tester to approve either the test
        was successfull or not.       
        """
        
    def registerComponent(self, componentName) -> None:
        """
        Resgisters the component to the ROS param server to be fetched later
        :param componentName: It should be the SAME name in the testable_components config      
        """