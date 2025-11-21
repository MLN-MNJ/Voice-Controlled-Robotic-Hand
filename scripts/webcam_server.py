# webcam_server.py
# --- RUN THIS ON YOUR WINDOWS MACHINE ---

import cv2
from flask import Flask, Response

app = Flask(__name__)
# We use 0 for the default webcam
video = cv2.VideoCapture(0) 

def gen_frames():  
    while True:
        success, frame = video.read()  # read the camera frame
        if not success:
            print("Webcam read failed")
            break
        else:
            # Encode the frame as JPEG
            ret, buffer = cv2.imencode('.jpg', frame)
            frame_bytes = buffer.tobytes()
            # Yield the frame in a format that a browser/client can understand
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')

@app.route('/video_feed')
def video_feed():
    # This is the endpoint our WSL 2 client will connect to
    return Response(gen_frames(), mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == '__main__':
    # Start the Flask server
    # '0.0.0.0' means it's accessible from any IP on your network
    print("Starting webcam server on http://<YOUR-WINDOWS-IP>:5000/video_feed")
    print("Find your IP using 'ipconfig' in CMD")
    app.run(host='0.0.0.0', port=5000)