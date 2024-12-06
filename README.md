# IoT_SmarCane
IoT4764


RPi:

1. source ~/my_env/bin/activate
2. python sensor.py
3. python process_data.py


GCP:
gunicorn -w 4 -b 0.0.0.0:5000 app:app


Website:
Sensor:
http://34.72.243.54:5000

Fall audios:
http://34.72.243.54:5000/fall_audios

