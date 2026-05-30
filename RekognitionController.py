import boto3
import cv2
import os

rekognition = boto3.client('rekognition', region_name=os.getenv('AWS_REGION', 'us-east-2'))

PHONE_LABELS = {'Cell Phone', 'Mobile Phone', 'Phone', 'Smartphone', 'Iphone', 'Ipod'}
DISTRACTION_LABELS = {'Game', 'Video Gaming', 'TV', 'Super Mario', 'VR Headset'}
MIN_CONFIDENCE = float(os.getenv('MIN_CONFIDENCE', '70'))


def check_for_phone(frame):
    _, buffer = cv2.imencode('.jpg', frame)
    image_bytes = buffer.tobytes()

    response = rekognition.detect_labels(
        Image={'Bytes': image_bytes},
        MaxLabels=20,
        MinConfidence=MIN_CONFIDENCE,
    )

    detected = {label['Name'] for label in response['Labels']}
    return bool(detected & PHONE_LABELS)


def check_for_distraction(frame):
    _, buffer = cv2.imencode('.jpg', frame)
    image_bytes = buffer.tobytes()

    response = rekognition.detect_labels(
        Image={'Bytes': image_bytes},
        MaxLabels=20,

        MinConfidence=MIN_CONFIDENCE,
    )

    detected = {label['Name'] for label in response['Labels']}
    
    return bool(detected & DISTRACTION_LABELS)