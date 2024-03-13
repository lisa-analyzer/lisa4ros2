import cv2
import numpy as np
import pkg_resources
from array import array
import threading
import json

from solar_interfaces.srv import DeliverImg
from .poly_dust_net import Unet_Model

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup


from rclpy.node import Node

_RGB = 1
_image_size = (1792, 1024)
class PolyDustService(Node):

    def __init__(self):
        super().__init__('poly_dust_service')
        self.callback_group = MutuallyExclusiveCallbackGroup()
        # Cargamos el modelo unet
        model_path = pkg_resources.resource_filename('poly_dust', 'sm_unet4_03.hdf5')
        self.unet_model = Unet_Model(model_path)
        # Servidor que recibe las peticiones de procesar las imagenes
        self.srv = self.create_service(DeliverImg, 
                                       'poly_dust_service', 
                                       self.proccess_img,
                                       callback_group=self.callback_group)
        # Cliente de deliver 
        
        self.get_logger().info('RUNING ...')


    def send_deliver_request(self, id):
        deliver_client = self.create_client(DeliverImg , 
                                            'deliver_server')
        while not deliver_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('deliver_server no disponible, Esperando ...')
        deliver_request = DeliverImg.Request()

        deliver_request.photo_id = id

        return deliver_client.call(deliver_request)
    
    def recover_img_from_deliver_response(self, response):
        img_bytes = np.array(response.photo, dtype=np.uint8)
        return cv2.imdecode(img_bytes, _RGB)

    def proccess_img(self, request, response):
        response.photo = []
        self.get_logger().info(f'Incoming request for poly dust server, photo with id {request.photo_id} asking delivery')
        # Hacemos una peticion al deliver
        stored_photo = self.send_deliver_request(request.photo_id)
        # Vemos si obtenemos una peticion exitosa
        if stored_photo.photo == array('B'):
            self.get_logger().info(f'Photo with id {request.photo_id} not avaible')
            return response
        self.get_logger().info(f'Recovered photo with id {request.photo_id}, using model')
        # Transformamos el formato a cv2  y la pasamos por poly_dust net
        image =  self.recover_img_from_deliver_response(stored_photo)
        image = cv2.resize(image, _image_size, interpolation = cv2.INTER_AREA)

        dust_image = self.unet_model.unet_prediction(image)

        self.get_logger().info(f'Resnet process eneded for photo with id {request.photo_id}')
        
        reconstructed = dust_image.astype(np.uint8) * 100
        reconstructed = array('B', reconstructed.tobytes())
        response.photo = reconstructed
        return response


def main():
    rclpy.init()
    try:
        poly_dust_service = PolyDustService()
        executor = MultiThreadedExecutor()
        executor.add_node(poly_dust_service)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            poly_dust_service.destroy_node()

    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()