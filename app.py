#!/usr/bin/env python3
"""
Estación de Monitoreo MAVLink - Backend Flask
Proyecto de Fin de Ciclo de Bryan P.A.

Backend completo para comunicación MAVLink con frontend web modular.
Soporta conexiones serie, TCP, UDP y modo simulación.
"""

from flask import Flask, render_template, jsonify, request
from flask_cors import CORS
import pymavlink.mavutil as mavutil
from pymavlink import mavutil
import threading
import time
import json
import math
import random
from datetime import datetime
import logging
import os

# Configurar logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Inicializar Flask con carpetas estáticas
app = Flask(__name__, 
           static_folder='static',
           static_url_path='/static')
CORS(app)

class MAVLinkManager:
    """Gestor de conexiones MAVLink con soporte para múltiples tipos de conexión"""
    
    def __init__(self):
        self.connection = None
        self.connected = False
        self.thread_running = False
        self.data_thread = None
        self.simulate_mode = False
        
        # Datos de telemetría
        self.telemetry_data = {
            'timestamp': None,
            'altitude': None,
            'groundspeed': None,
            'airspeed': None,
            'heading': None,
            'lat': None,
            'lon': None,
            'battery_voltage': None,
            'battery_remaining': None,
            'flight_mode': 'UNKNOWN',
            'armed': False,
            'gps_status': 'NO_GPS',
            'satellites_visible': 0,
            'roll': None,
            'pitch': None,
            'yaw': None,
            'vertical_speed': None,
            'throttle': None
        }
        
        # Variables para simulación
        self.sim_time = 0
        self.sim_altitude = 0
        self.sim_lat = 43.35100
        self.sim_lon = -8.41834
        
    def connect(self, connection_type='serial', **kwargs):
        """Establecer conexión MAVLink"""
        try:
            if self.connected:
                self.disconnect()
                
            if connection_type == 'simulate':
                self.simulate_mode = True
                self.connected = True
                logger.info("Iniciando modo simulación")
            else:
                self.simulate_mode = False
                connection_string = self._build_connection_string(connection_type, **kwargs)
                
                logger.info(f"Conectando a: {connection_string}")
                self.connection = mavutil.mavlink_connection(connection_string)
                
                # Esperar heartbeat
                logger.info("Esperando heartbeat...")
                self.connection.wait_heartbeat(timeout=10)
                logger.info("Heartbeat recibido!")
                
                self.connected = True
            
            # Iniciar hilo de lectura de datos
            self.thread_running = True
            self.data_thread = threading.Thread(target=self._data_loop, daemon=True)
            self.data_thread.start()
            
            return True, "Conexión establecida"
            
        except Exception as e:
            logger.error(f"Error de conexión: {e}")
            self.connected = False
            return False, str(e)
    
    def _build_connection_string(self, connection_type, **kwargs):
        """Construir string de conexión según el tipo"""
        if connection_type == 'serial':
            port = kwargs.get('port', '/dev/ttyACM0')
            baud = kwargs.get('baud', 115200)
            return f"{port}:{baud}"
        elif connection_type == 'tcp':
            host = kwargs.get('host', '10.207.0.112')
            port = kwargs.get('port', 2001)
            return f"tcp:{host}:{port}"
        elif connection_type == 'udp':
            host = kwargs.get('host', '10.207.0.112')
            port = kwargs.get('port', 14550)
            return f"udp:{host}:{port}"
        else:
            raise ValueError(f"Tipo de conexión no soportado: {connection_type}")
    
    def disconnect(self):
        """Desconectar MAVLink"""
        try:
            self.thread_running = False
            self.connected = False
            
            if self.data_thread and self.data_thread.is_alive():
                self.data_thread.join(timeout=2)
            
            if self.connection:
                self.connection.close()
                self.connection = None
                
            # Reset data
            for key in self.telemetry_data:
                if key == 'flight_mode':
                    self.telemetry_data[key] = 'UNKNOWN'
                elif key == 'gps_status':
                    self.telemetry_data[key] = 'NO_GPS'
                elif key in ['armed']:
                    self.telemetry_data[key] = False
                else:
                    self.telemetry_data[key] = None
                    
            logger.info("Desconectado correctamente")
            return True, "Desconectado"
            
        except Exception as e:
            logger.error(f"Error al desconectar: {e}")
            return False, str(e)
    
    def _data_loop(self):
        """Hilo principal de lectura de datos"""
        logger.info("Iniciando hilo de datos")
        
        while self.thread_running:
            try:
                if self.simulate_mode:
                    self._simulate_data()
                else:
                    self._read_mavlink_data()
                    
                time.sleep(0.1)  # 10Hz
                
            except Exception as e:
                logger.error(f"Error en hilo de datos: {e}")
                time.sleep(1)
        
        logger.info("Hilo de datos terminado")
    
    def _simulate_data(self):
        """Generar datos simulados"""
        self.sim_time += 0.1
        
        # Simular vuelo en círculo
        self.sim_altitude = 50 + 20 * math.sin(self.sim_time * 0.1)
        
        # Movimiento GPS circular
        radius = 0.001  # ~100m
        self.sim_lat = 43.35100 + radius * math.cos(self.sim_time * 0.01)
        self.sim_lon = -8.41834 + radius * math.sin(self.sim_time * 0.01)
        
        # Actualizar datos simulados
        self.telemetry_data.update({
            'timestamp': datetime.now().isoformat(),
            'altitude': round(self.sim_altitude, 1),
            'groundspeed': round(15 + 5 * math.sin(self.sim_time * 0.2), 1),
            'airspeed': round(18 + 3 * math.cos(self.sim_time * 0.15), 1),
            'heading': int((self.sim_time * 5) % 360),
            'lat': round(self.sim_lat, 7),
            'lon': round(self.sim_lon, 7),
            'battery_voltage': round(22.2 - (self.sim_time * 0.01), 1),
            'battery_remaining': max(0, int(100 - (self.sim_time * 0.5))),
            'flight_mode': 'AUTO',
            'armed': True,
            'gps_status': 'GPS_OK',
            'satellites_visible': 12,
            'roll': round(10 * math.sin(self.sim_time * 0.3), 1),
            'pitch': round(5 * math.cos(self.sim_time * 0.25), 1),
            'yaw': round((self.sim_time * 5) % 360, 1),
            'vertical_speed': round(2 * math.cos(self.sim_time * 0.1), 1),
            'throttle': int(50 + 20 * math.sin(self.sim_time * 0.2))
        })
    
    def _read_mavlink_data(self):
        """Leer datos reales de MAVLink"""
        if not self.connection:
            return
            
        try:
            msg = self.connection.recv_match(timeout=1)
            if msg is None:
                return
                
            msg_type = msg.get_type()
            
            # Procesar diferentes tipos de mensajes
            if msg_type == 'GLOBAL_POSITION_INT':
                self.telemetry_data.update({
                    'lat': msg.lat / 1e7,
                    'lon': msg.lon / 1e7,
                    'altitude': msg.relative_alt / 1000.0,
                    'heading': msg.hdg / 100.0,
                    'vertical_speed': msg.vz / 100.0
                })
                
            elif msg_type == 'VFR_HUD':
                self.telemetry_data.update({
                    'groundspeed': msg.groundspeed,
                    'airspeed': msg.airspeed,
                    'heading': msg.heading,
                    'throttle': msg.throttle,
                    'altitude': msg.alt
                })
                
            elif msg_type == 'SYS_STATUS':
                voltage = msg.voltage_battery / 1000.0 if msg.voltage_battery != 65535 else None
                remaining = msg.battery_remaining if msg.battery_remaining != -1 else None
                
                self.telemetry_data.update({
                    'battery_voltage': voltage,
                    'battery_remaining': remaining
                })
                
            elif msg_type == 'GPS_RAW_INT':
                # Estados GPS según MAVLink
                gps_status_map = {
                    0: 'NO_GPS',
                    1: 'NO_FIX',
                    2: '2D_FIX',
                    3: '3D_FIX',
                    4: 'DGPS',
                    5: 'RTK_FLOAT',
                    6: 'RTK_FIXED'
                }
                
                self.telemetry_data.update({
                    'gps_status': gps_status_map.get(msg.fix_type, 'UNKNOWN'),
                    'satellites_visible': msg.satellites_visible
                })
                
            elif msg_type == 'ATTITUDE':
                self.telemetry_data.update({
                    'roll': math.degrees(msg.roll),
                    'pitch': math.degrees(msg.pitch),
                    'yaw': math.degrees(msg.yaw)
                })
                
            elif msg_type == 'HEARTBEAT':
                # Decodificar modo de vuelo
                flight_modes = {
                    0: 'MANUAL', 1: 'CIRCLE', 2: 'STABILIZE', 3: 'TRAINING',
                    4: 'ACRO', 5: 'FBWA', 6: 'FBWB', 7: 'CRUISE',
                    8: 'AUTOTUNE', 10: 'AUTO', 11: 'RTL', 12: 'LOITER',
                    15: 'GUIDED', 16: 'INITIALISING'
                }
                
                mode = flight_modes.get(msg.custom_mode, f'MODE_{msg.custom_mode}')
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                
                self.telemetry_data.update({
                    'flight_mode': mode,
                    'armed': armed
                })
            
            # Actualizar timestamp
            self.telemetry_data['timestamp'] = datetime.now().isoformat()
                
        except Exception as e:
            logger.error(f"Error procesando mensaje MAVLink: {e}")

# Instancia global del gestor MAVLink
mavlink_manager = MAVLinkManager()

# Rutas Flask
@app.route('/')
def index():
    """Página principal"""
    return render_template('index.html')

@app.route('/api/connect', methods=['POST'])
def api_connect():
    """Endpoint para conectar al dron"""
    try:
        data = request.get_json()
        connection_type = data.get('connection_type', 'serial')
        
        # Parámetros según tipo de conexión
        params = {}
        if connection_type in ['serial', 'simulate']:
            params['port'] = data.get('port', '/dev/ttyACM0')
            params['baud'] = data.get('baud', 115200)
        else:
            params['host'] = data.get('host', '10.207.0.112')
            params['port'] = data.get('port', 14550)
        
        success, message = mavlink_manager.connect(connection_type, **params)
        
        return jsonify({
            'success': success,
            'message': message,
            'connection_type': connection_type
        })
        
    except Exception as e:
        logger.error(f"Error en /api/connect: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/disconnect', methods=['POST'])
def api_disconnect():
    """Endpoint para desconectar del dron"""
    try:
        success, message = mavlink_manager.disconnect()
        return jsonify({'success': success, 'message': message})
        
    except Exception as e:
        logger.error(f"Error en /api/disconnect: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

@app.route('/api/status')
def api_status():
    """Endpoint para obtener estado de conexión"""
    return jsonify({
        'connected': mavlink_manager.connected,
        'simulate_mode': mavlink_manager.simulate_mode
    })

@app.route('/api/telemetry')
def api_telemetry():
    """Endpoint para obtener datos de telemetría"""
    try:
        return jsonify({
            'success': True,
            'data': mavlink_manager.telemetry_data,
            'timestamp': datetime.now().isoformat()
        })
        
    except Exception as e:
        logger.error(f"Error en /api/telemetry: {e}")
        return jsonify({'success': False, 'error': str(e)}), 500

if __name__ == '__main__':
    # Verificar si la carpeta estática existe
    if not os.path.exists('static'):
        os.makedirs('static');
    
    print("=" * 70)
    print("ESTACIÓN DE MONITOREO MAVLINK - PROYECTO FIN DE CICLO DE BRYAN P.A.")
    print("=" * 70)
    print(f"Servidor iniciado en: http://Hostname(Esto depende de la ip a la que estas conectado):3001")
    print(f"Archivos estáticos en: static/")
    print(f"API endpoints disponibles:")
    print(f"   • POST /api/connect    - Conectar al dron")
    print(f"   • POST /api/disconnect - Desconectar del dron")
    print(f"   • GET  /api/status     - Estado de conexión")
    print(f"   • GET  /api/telemetry  - Datos de telemetría")
    print("=" * 70)
    
    app.run(debug=True, host='10.207.0.112', port=3001)
