import socket
import threading
import speech_recognition as sr
import time
import sys

# --- CONFIGURATION ---
ESP32_IP = "192.168.1.166" 
PORT = 8888

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.settimeout(1.0)

def send_to_robot(final_cmd):
    try:
        sock.sendto(bytes(final_cmd + "\n", "utf-8"), (ESP32_IP, PORT))
    except Exception as e:
        print(f"\n[ERROR] {e}")

# --- 1. RECEIVE THREAD ---
def receive_listener():
    while True:
        try:
            data, addr = sock.recvfrom(1024)
            sys.stdout.write(f"\r[ROBOT]: {data.decode().strip()}\nCommand >> ")
            sys.stdout.flush()
        except:
            continue

# --- 2. THE OPTIMIZED VOICE THREAD ---
def voice_listener():
    recognizer = sr.Recognizer()
    mic = sr.Microphone()
    
    # --- CRITICAL TUNING ---
    recognizer.energy_threshold = 1200  # Higher = ignores more noise. Try 800-1500.
    recognizer.dynamic_energy_threshold = False  # Keep it static so motor noise doesn't confuse it
    recognizer.pause_threshold = 0.5    # How long to wait after you stop talking
    recognizer.non_speaking_duration = 0.3 # Keep this low for snappy response
    
    # ACTION KEYWORDS (Fuzzy Matching)
    DANCE_WORDS = ["dance", "dancing", "dancer", "moves", "party"]
    THINK_WORDS = ["think", "thinking", "thought", "brain", "ponder"]
    STOP_WORDS  = ["stop", "halt", "kill", "shut down", "relax"]

    with mic as source:
        print("\n[VOICE] Calibrating for background noise...")
        # Longer calibration helps filter out consistent fan/motor noise
        recognizer.adjust_for_ambient_noise(source, duration=3)
        print(f"[VOICE] Ready! Energy Floor: {recognizer.energy_threshold}")

    while True:
        with mic as source:
            try:
                # Use a small timeout so the loop stays alive
                audio = recognizer.listen(source, timeout=None, phrase_time_limit=4)
                
                # Transcribe
                text = recognizer.recognize_google(audio).lower()
                sys.stdout.write(f"\r[HEARD]: '{text}'\nCommand >> ")
                sys.stdout.flush()

                # --- FUZZY LOGIC ---
                if any(word in text for word in DANCE_WORDS):
                    print("!!! ACTION: LUCKY:DANCE")
                    send_to_robot("LUCKY:DANCE")
                
                elif any(word in text for word in THINK_WORDS):
                    print("!!! ACTION: LUCKY:THINK")
                    send_to_robot("LUCKY:THINK")
                
                elif any(word in text for word in STOP_WORDS):
                    print("!!! ACTION: STOPPING")
                    send_to_robot("BAL:OFF")

            except sr.UnknownValueError:
                # This triggers if it hears noise but no words
                continue
            except Exception as e:
                continue

# --- 3. MAIN CLI LOOP ---
if __name__ == "__main__":
    threading.Thread(target=receive_listener, daemon=True).start()
    threading.Thread(target=voice_listener, daemon=True).start()
    
    print("\nLUCKY ONLINE - (Fuzzy Voice Enabled)")
    try:
        while True:
            raw_input = input("Command >> ").strip().upper()
            if not raw_input: continue
            if raw_input == 'Q': break
            
            # Simple CLI Mapping
            parts = raw_input.split()
            cmd_type = parts[0]
            
            if cmd_type == 'P' and len(parts) == 3:
                send_to_robot(f"POS:{parts[1]}:{parts[2]}")
            elif cmd_type == 'V' and len(parts) == 3:
                send_to_robot(f"VEL:{parts[1]}:{parts[2]}")
            elif cmd_type == 'Z':
                send_to_robot("ZERO:ALL")
            else:
                send_to_robot(raw_input)
                
    except KeyboardInterrupt:
        pass