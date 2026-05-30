import io
import os
from boxsdk import Client, OAuth2


def upload_to_box(image_bytes: bytes, filename: str) -> str:
    auth = OAuth2(
        client_id=os.environ['BOX_CLIENT_ID'],
        client_secret=os.environ['BOX_CLIENT_SECRET'],
        access_token=os.environ['BOX_DEVELOPER_TOKEN'],
    )
    client = Client(auth)
    folder_id = os.getenv('BOX_FOLDER_ID', '0')

    uploaded = client.folder(folder_id).upload_stream(
        io.BytesIO(image_bytes), filename
    )
    return f"https://app.box.com/file/{uploaded.id}"
