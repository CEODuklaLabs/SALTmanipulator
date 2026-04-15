"""
SALT Manipulator – entry point
Run: python main.py  →  http://localhost:5000
"""

from webapp import app
import config

if __name__ == "__main__":
    print(f"SALT Manipulator → http://localhost:5000  (Arduino: {config.ARDUINO_PORT})")
    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
