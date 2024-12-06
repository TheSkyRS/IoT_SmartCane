# IoT_SmarCane
IoT4764


RPi:

1. source ~/my_env/bin/activate
2. python sensor.py
3. python process_data.py


GCP:
gunicorn -w 4 -b 0.0.0.0:5000 flask_server:app

