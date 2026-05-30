import boto3
import os

ses = boto3.client('ses', region_name=os.getenv('AWS_REGION', 'us-east-2'))

def send_alert_email(filename: str, box_url: str | None = None) -> None:
    sender = os.environ['SES_SENDER_EMAIL']
    recipient = os.environ['ALERT_EMAIL']

    body_lines = [
        "RoombaRat has caught a phone on campus!",
        "",
        f"Evidence file: {filename}",
    ]
    if box_url:
        body_lines.append(f"View image: {box_url}")

    ses.send_email(
        Source=sender,
        Destination={'ToAddresses': [recipient]},
        Message={
            'Subject': {'Data': 'BUSTED: Phone Detected on Campus'},
            'Body': {'Text': {'Data': '\n'.join(body_lines)}},
        },
    )

def send_distraction_email(filename: str, box_url: str | None = None) -> None:
    sender = os.environ['SES_SENDER_EMAIL']
    recipient = os.environ['ALERT_EMAIL']

    body_lines = [
        "RoombaRat has caught some slacking off on campus!",
        "",
        f"Evidence file: {filename}",
    ]
    if box_url:
        body_lines.append(f"View image: {box_url}")

    ses.send_email(
        Source=sender,
        Destination={'ToAddresses': [recipient]},
        Message={
            'Subject': {'Data': 'BUSTED: Distraction Detected on Campus'},
            'Body': {'Text': {'Data': '\n'.join(body_lines)}},
        },
    )
