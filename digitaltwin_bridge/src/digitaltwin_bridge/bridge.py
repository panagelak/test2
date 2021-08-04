#!/usr/bin/env python
import rospy
import paho.mqtt.client as mqtt
from rospy_message_converter import json_message_converter

from integration.msg import PclTransfer

class MqttRosBridge(object):
    def __init__(self):
        # mqtt prerequisities
        self._broker = rospy.get_param("~address") # "150.140.148.252"
        self._port = rospy.get_param("~port") # "8883"
        self._sender = rospy.get_param("~sender") 
        print(self._broker)

        # Client send pcl
        if self._sender == True:
            self._client_send_pcl = mqtt.Client("client-send_pcl")
            self._client_send_pcl.on_connect = self.on_connect
            self._client_send_pcl.on_log = self.on_log
            self._client_send_pcl_sub_ = rospy.Subscriber(
                "/transfer_pcl", PclTransfer, self.send_pcl_callback)
            self._client_send_pcl.connect(
                host=self._broker, port=self._port)
            self._client_send_pcl.loop_start()

        # Client receive pcl
        if self._sender == False:
            self._client_receive_pcl = mqtt.Client(
                "client-receive_pcl")
            self._client_receive_pcl.on_connect = self.on_connect
            self._client_receive_pcl.on_log = self.on_log
            self._client_receive_pcl.on_message = self.on_receive_pcl
            self._client_receive_pcl.connect(
                host=self._broker, port=self._port)
            self._client_receive_pcl.loop_start()
            self._client_receive_pcl.subscribe("transfer_pcl")
            self._receive_pcl_pub = rospy.Publisher(
                "/compress_pcl", PclTransfer, queue_size=100, latch=True)


    def __del__(self):
        if self._sender == True:
            self._client_send_pcl.loop_stop()
            self._client_send_pcl.disconnect()
        if self._sender == False:
            self._client_receive_pcl.loop_stop()
            self._client_receive_pcl.disconnect()

    def send_pcl_callback(self, data):
        if self._sender == True:
            json_data = json_message_converter.convert_ros_message_to_json(data)
            self._client_send_pcl.publish("transfer_pcl", json_data)

    def on_receive_pcl(self, client, userdata, message):
        if self._sender == False:
            ros_data = json_message_converter.convert_json_to_ros_message(
                'integration/PclTransfer', message.payload)
            self._receive_pcl_pub.publish(ros_data)


    def on_log(self, client, userdata, level, buf):
        print("log: "+buf)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("connected OK")
        else:
            print("Bad connection Returned code=", rc)


__all__ = ['MqttRosBridge']
