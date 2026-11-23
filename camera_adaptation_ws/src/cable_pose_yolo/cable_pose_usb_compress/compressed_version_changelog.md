# Registro de Cambios: Versión Normal vs. Versión Compress

Este documento detalla las modificaciones técnicas realizadas en el archivo `detector_node.py` para optimizar el consumo de CPU (en sistemas sin GPU dedicada) y reducir drásticamente el ancho de banda de red en la salida de la imagen anotada.

## 1. Importación del tipo de mensaje comprimido
Se ha añadido `CompressedImage` a las dependencias de ROS 2 para poder estructurar y enviar mensajes empaquetados en JPEG en lugar de usar matrices planas de píxeles (RAW).
* **Antes:** `from sensor_msgs.msg import Image, CameraInfo`
* **Después:** `from sensor_msgs.msg import Image, CameraInfo, CompressedImage`

## 2. Implementación de limitador temporal (Throttling)
Para evitar la saturación del procesador (CPU al ~800%), se ha introducido un filtro que descarta fotogramas en el `image_callback`, bajando la tasa de procesamiento de ~30 FPS a ~10 FPS.
* **Añadido en `__init__`:**
  ```python
  self.frame_count = 0
  self.process_every_n_frames = 3  # Procesa 1 de cada 3 frames (~10 FPS)
  ```
* **Añadido en `image_callback`:**
  ```python
  self.frame_count += 1
  if self.frame_count % self.process_every_n_frames != 0:
      return
  ```

## 3. Reducción de la resolución de inferencia (YOLO)
Se ha modificado la llamada de inferencia para forzar a YOLO a procesar las imágenes a 320x320 píxeles. Al no utilizar la resolución nativa (640x640), el esfuerzo matemático de la CPU se divide drásticamente.
* **Antes:** `resultados = self.model(cv_image, conf=self.conf_thresh, verbose=False)`
* **Después:** `resultados = self.model(cv_image, conf=self.conf_thresh, imgsz=320, verbose=False)`

## 4. Modificación del Publicador de Depuración
El publicador encargado de emitir la imagen resultante (con las cajas delimitadoras dibujadas) se ha adaptado para utilizar el nuevo tipo de mensaje y renombrar el tópico siguiendo el estándar de ROS 2.
* **Antes:** 
  ```python
  self.image_pub = self.create_publisher(Image, '/camera/color/image_annotated', 10)
  ```
* **Después:** 
  ```python
  self.image_pub = self.create_publisher(CompressedImage, '/camera/color/image_annotated/compressed', 10)
  ```

## 5. Codificación manual a formato JPEG
Se ha eliminado el uso estándar de `cv_bridge` para publicar la imagen. En su lugar, se emplea OpenCV directamente para comprimir la matriz de píxeles a formato JPEG (con calidad al 80%), generando una cadena de bytes ultraligera.
* **Antes (cv_bridge):**
  ```python
  msg_annotated = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
  self.image_pub.publish(msg_annotated)
  ```
* **Después (OpenCV encode):**
  ```python
  msg_annotated = CompressedImage()
  msg_annotated.header = msg.header
  msg_annotated.format = "jpeg"
  success, encoded_image = cv2.imencode('.jpg', annotated_frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
  if success:
      msg_annotated.data = encoded_image.tobytes()
      self.image_pub.publish(msg_annotated)
  ```