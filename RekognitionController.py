import boto3
import base64
import cv2

rekognition = boto3.client('rekognition', region_name='us-east-1')

def check_for_phone(frame):
    # encode opencv frame as JPEG bytes
    _, buffer = cv2.imencode('.jpg', frame)
    image_bytes = buffer.tobytes()
    
    response = rekognition.detect_labels(
        Image={'Bytes': image_bytes},
        MaxLabels=20,
        MinConfidence=70  # tune this
    )
    
    labels = [label['Name'] for label in response['Labels']]
    return 'Cell Phone' in labels