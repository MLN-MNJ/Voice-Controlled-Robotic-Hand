# windows_voice_server.py
# --- RUN THIS ON WINDOWS (PowerShell/CMD) ---

from flask import Flask, jsonify
import speech_recognition as sr
import threading
import time

app = Flask(__name__)

# Global variable to store the last spoken text
last_command = ""
command_lock = threading.Lock()

def listen_in_background():
    global last_command
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    
    with mic as source:
        print("Adjusting for ambient noise...")
        recognizer.adjust_for_ambient_noise(source)
        print("Microphone listening on Windows side...")
    
    while True:
        try:
            with mic as source:
                # Listen for audio
                audio = recognizer.listen(source)
                
                try:
                    # Convert to text
                    text = recognizer.recognize_google(audio).lower()
                    print(f" heard: '{text}'")
                    
                    # Save it safely
                    with command_lock:
                        last_command = text
                        
                except sr.UnknownValueError:
                    pass # Ignored unintelligible speech
                except sr.RequestError:
                    print("API Error")
                    
        except Exception as e:
            print(f"Error in listener: {e}")
            time.sleep(1)

# Route for WSL to "ask" for the latest command
@app.route('/get_command', methods=['GET'])
def get_command():
    global last_command
    with command_lock:
        response = last_command
        last_command = "" # Clear it after reading so we don't repeat it
    return jsonify({'command': response})

if __name__ == '__main__':
    # Start the listener in a separate thread
    t = threading.Thread(target=listen_in_background)
    t.daemon = True
    t.start()
    
    # Start the web server on Port 5001 (to not conflict with webcam on 5000)
    app.run(host='0.0.0.0', port=5001)