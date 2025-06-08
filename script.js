#!/usr/bin/env python3
"""
MAVLink Flask Backend Server
Proyecto de Fin de Ciclo - Estación de Monitoreo MAVLink

Este servidor Flask proporciona una API REST para la comunicación
con drones MAVLink y sirve los datos de telemetría al frontend.

Funcionalidades:
- Conexión MAVLink (Serial, UDP, TCP, Simulación)
- API REST para telemetría en tiempo real
- Manejo de múltiples tipos de mensajes MAVLink
- Sistema de logging y manejo de errores
- Soporte para simulación de datos
"""

from flask import Flask, request, jsonify, render_template
from flask_cors import CORS
from pymavlink import mavutil
import threading
import time
import json
import logging
import math
import random
from datetime import datetime
from typing import Optional, Dict, Any

# Configuración de logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

class MAVLinkConnection:
    """Clase para manejar la conexión MAVLink"""
    
    def __init__(self):
        self.master: Optional[mavutil.mavlink_connection] = None
        self.connected = False
        self.telemetry_thread: Optional[threading.Thread] = None
        self.stop_telemetry = False
        self.last_telemetry = {}
        self.connection_type = None
        self.simulate_mode = False
        
        # Datos de simulación para A Coruña
        self.sim_data = {
            'lat': 43.351314892682474,
            'lon': -8.418597088780944,
            'altitude': 0.0,
            'groundspeed': 0.0,
            'heading': 0.0,
            'battery_remaining': 100.0,
            'flight_mode': 'STABILIZE',
            'armed': False,
            'gps_status': 3,
            'satellites_visible': 12,
            'roll': 0.0,
            'pitch': 0.0,
            'yaw': 0.0
        }
        
    def connect(self, connection_type: str, **kwargs) -> bool:
        """
        Conectar al dron MAVLink
        
        Args:
            connection_type: 'serial', 'udp', 'tcp', 'simulate'
            **kwargs: Parámetros específicos de conexión
        """
        try:
            self.connection_type = connection_type
            
            if connection_type == 'simulate':
                logger.info("Iniciando modo de simulación")
                self.simulate_mode = True
                self.connected = True
                self.start_telemetry_thread()
                return True
            
            # Construir string de conexión
            if connection_type == 'serial':
                port = kwargs.get('port', '/dev/ttyACM0')
                baud = kwargs.get('baud', 57600)
                connection_string = f"{port}"
                logger.info(f"Conectando por serial: {port} @ {baud}")
                self.master = mavutil.mavlink_connection(connection_string, baud=baud)
                
            elif connection_type == 'udp':
                host = kwargs.get('host', 'localhost')
                port = kwargs.get('port', 14550)
                connection_string = f"udp:{host}:{port}"
                logger.info(f"Conectando por UDP: {connection_string}")
                self.master = mavutil.mavlink_connection(connection_string)
                
            elif connection_type == 'tcp':
                host = kwargs.get('host', 'localhost')
                port = kwargs.get('port', 5760)
                connection_string = f"tcp:{host}:{port}"
                logger.info(f"Conectando por TCP: {connection_string}")
                self.master = mavutil.mavlink_connection(connection_string)
            
            # Esperar heartbeat
            logger.info("Esperando heartbeat...")
            self.master.wait_heartbeat(timeout=10)
            logger.info("Heartbeat recibido - Conexión establecida")
            
            self.connected = True
            self.simulate_mode = False
            self.start_telemetry_thread()
            return True
            
        except Exception as e:
            logger.error(f"Error de conexión: {e}")
            self.connected = False
            return False
    
    def disconnect(self) -> bool:
        """Desconectar del dron"""
        try:
            self.stop_telemetry_thread()
            
            if self.master:
                self.master.close()
                self.master = None
                
            self.connected = False
            self.simulate_mode = False
            logger.info("Desconectado exitosamente")
            return True
            
        except Exception as e:
            logger.error(f"Error al desconectar: {e}")
            return False
    
    def start_telemetry_thread(self):
        """Iniciar hilo de telemetría"""
        self.stop_telemetry = False
        self.telemetry_thread = threading.Thread(target=self._telemetry_worker)
        self.telemetry_thread.daemon = True
        self.telemetry_thread.start()
        logger.info("Hilo de telemetría iniciado")
    
    def stop_telemetry_thread(self):
        """Detener hilo de telemetría"""
        self.stop_telemetry = True
        if self.telemetry_thread and self.telemetry_thread.is_alive():
            self.telemetry_thread.join(timeout=2)
        logger.info("Hilo de telemetría detenido")
    
    def _telemetry_worker(self):
        """Worker para recolectar telemetría"""
        logger.info("Iniciando recolección de telemetría")
        
        while not self.stop_telemetry and self.connected:
            try:
                if self.simulate_mode:
                    self._update_simulation_data()
                else:
                    self._collect_real_telemetry()
                
                time.sleep(0.1)  # 10Hz de actualización
                
            except Exception as e:
                logger.error(f"Error en telemetría: {e}")
                time.sleep(1)
    
    def _update_simulation_data(self):
        """Actualizar datos de simulación"""
        # Simular movimiento ligero del dron
        self.sim_data['altitude'] += random.uniform(-0.5, 0.5)
        self.sim_data['altitude'] = max(0, self.sim_data['altitude'])
        
        # Simular ligero movimiento GPS
        self.sim_data['lat'] += random.uniform(-0.00001, 0.00001)
        self.sim_data['lon'] += random.uniform(-0.00001, 0.00001)
        
        # Simular cambios de heading
        self.sim_data['heading'] += random.uniform(-2, 2)
        self.sim_data['heading'] = self.sim_data['heading'] % 360
        
        # Simular actitud
        self.sim_data['roll'] = random.uniform(-5, 5)
        self.sim_data['pitch'] = random.uniform(-3, 3)
        self.sim_data['yaw'] = self.sim_data['heading']
        
        # Simular velocidad
        self.sim_data['groundspeed'] = random.uniform(0, 2)
        
        # Simular batería (descarga lenta)
        if self.sim_data['battery_remaining'] > 50:
            self.sim_data['battery_remaining'] -= random.uniform(0, 0.01)
        
        self.last_telemetry = self.sim_data.copy()
        self.last_telemetry['timestamp'] = time.time()
    
    def _collect_real_telemetry(self):
        """Recolectar telemetría real del dron"""
        if not self.master:
            return
        
        # Recolectar diferentes tipos de mensajes
        msg_types = [
            'GLOBAL_POSITION_INT', 'GPS_RAW_INT', 'ATTITUDE', 
            'VFR_HUD', 'SYS_STATUS', 'HEARTBEAT'
        ]
        
        telemetry_data = self.last_telemetry.copy()
        
        for msg_type in msg_types:
            try:
                msg = self.master.recv_match(type=msg_type, blocking=False, timeout=0.1)
                if msg:
                    self._process_mavlink_message(msg, telemetry_data)
            except Exception as e:
                logger.debug(f"Error procesando {msg_type}: {e}")
        
        # Actualizar timestamp
        telemetry_data['timestamp'] = time.time()
        self.last_telemetry = telemetry_data
    
    def _process_mavlink_message(self, msg, telemetry_data: Dict[str, Any]):
        """Procesar mensaje MAVLink específico"""
        msg_type = msg.get_type()
        
        try:
            if msg_type == 'GLOBAL_POSITION_INT':
                telemetry_data['lat'] = msg.lat / 1e7
                telemetry_data['lon'] = msg.lon / 1e7
                telemetry_data['altitude'] = msg.relative_alt / 1000.0  # mm to m
                telemetry_data['heading'] = msg.hdg / 100.0  # centidegrees to degrees
                
            elif msg_type == 'GPS_RAW_INT':
                telemetry_data['gps_status'] = msg.fix_type
                telemetry_data['satellites_visible'] = msg.satellites_visible
                if msg.fix_type >= 3:  # Solo usar GPS si hay fix 3D
                    telemetry_data['lat'] = msg.lat / 1e7
                    telemetry_data['lon'] = msg.lon / 1e7
                    telemetry_data['altitude'] = msg.alt / 1000.0  # mm to m
                
            elif msg_type == 'ATTITUDE':
                telemetry_data['roll'] = math.degrees(msg.roll)
                telemetry_data['pitch'] = math.degrees(msg.pitch)
                telemetry_data['yaw'] = math.degrees(msg.yaw)
                
            elif msg_type == 'VFR_HUD':
                telemetry_data['groundspeed'] = msg.groundspeed
                telemetry_data['altitude'] = msg.alt
                telemetry_data['heading'] = msg.heading
                
            elif msg_type == 'SYS_STATUS':
                telemetry_data['battery_remaining'] = msg.battery_remaining
                
            elif msg_type == 'HEARTBEAT':
                # Determinar modo de vuelo
                custom_mode = msg.custom_mode
                telemetry_data['armed'] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                
                # Mapear modos de vuelo básicos (específico para ArduPilot)
                flight_modes = {
                    0: 'STABILIZE', 1: 'ACRO', 2: 'ALT_HOLD', 3: 'AUTO',
                    4: 'GUIDED', 5: 'LOITER', 6: 'RTL', 7: 'CIRCLE',
                    8: 'POSITION', 9: 'LAND', 10: 'OF_LOITER', 11: 'DRIFT'
                }
                telemetry_data['flight_mode'] = flight_modes.get(custom_mode, f'MODE_{custom_mode}')
                
        except Exception as e:
            logger.debug(f"Error procesando mensaje {msg_type}: {e}")
    
    def get_telemetry(self) -> Dict[str, Any]:
        """Obtener última telemetría"""
        if not self.last_telemetry:
            return {
                'lat': None, 'lon': None, 'altitude': None,
                'groundspeed': None, 'heading': None, 'battery_remaining': None,
                'flight_mode': None, 'armed': None, 'gps_status': None,
                'satellites_visible': None, 'roll': None, 'pitch': None, 'yaw': None
            }
        
        return self.last_telemetry.copy()

# Inicializar Flask
app = Flask(__name__)
CORS(app)  # Permitir CORS para desarrollo

# Instancia global de conexión MAVLink
mavlink_conn = MAVLinkConnection()

@app.route('/')
def index():
    """Página principal"""
    return '''
    <!DOCTYPE html>
    <html>
    <head>
        <title>MAVLink Dashboard API</title>
        <style>
            body { font-family: Arial, sans-serif; margin: 40px; }
            .status { padding: 10px; border-radius: 5px; margin: 10px 0; }
            .connected { background-color: #d4edda; color: #155724; }
            .disconnected { background-color: #f8d7da; color: #721c24; }
            code { background-color: #f8f9fa; padding: 2px 4px; border-radius: 3px; }
        </style>
    </head>
    <body>
        <h1>🚁 MAVLink Dashboard API</h1>
        <p>Servidor Flask para comunicación MAVLink</p>
        
        <div id="status" class="status disconnected">
            ❌ Desconectado
        </div>
        
        <h2>Endpoints Disponibles:</h2>
        <ul>
            <li><code>POST /api/connect</code> - Conectar al dron</li>
            <li><code>POST /api/disconnect</code> - Desconectar del dron</li>
            <li><code>GET /api/telemetry</code> - Obtener telemetría</li>
            <li><code>GET /api/status</code> - Estado de conexión</li>
        </ul>
        
        <h2>Ejemplo de Conexión:</h2>
        <pre><code>POST /api/connect
{
    "connection_type": "simulate"
}</code></pre>
        
        <script>
            function updateStatus() {
                fetch('/api/status')
                    .then(response => response.json())
                    .then(data => {
                        const statusDiv = document.getElementById('status');
                        if (data.connected) {
                            statusDiv.className = 'status connected';
                            statusDiv.innerHTML = '✅ Conectado (' + data.connection_type + ')';
                        } else {
                            statusDiv.className = 'status disconnected';
                            statusDiv.innerHTML = '❌ Desconectado';
                        }
                    })
                    .catch(error => console.error('Error:', error));
            }
            
            // Actualizar estado cada 2 segundos
            setInterval(updateStatus, 2000);
            updateStatus();
        </script>
    </body>
    </html>
    '''

@app.route('/api/connect', methods=['POST'])
def connect():
    """Endpoint para conectar al dron"""
    try:
        data = request.get_json()
        connection_type = data.get('connection_type', 'simulate')
        
        logger.info(f"Solicitud de conexión: {connection_type}")
        
        # Parámetros según tipo de conexión
        kwargs = {}
        if connection_type == 'serial':
            kwargs['port'] = data.get('port', '/dev/ttyACM0')
            kwargs['baud'] = data.get('baud', 57600)
        elif connection_type in ['udp', 'tcp']:
            kwargs['host'] = data.get('host', 'localhost')
            kwargs['port'] = data.get('port', 14550 if connection_type == 'udp' else 5760)
        
        # Intentar conexión
        success = mavlink_conn.connect(connection_type, **kwargs)
        
        if success:
            logger.info("Conexión establecida exitosamente")
            return jsonify({
                'success': True,
                'message': f'Conectado via {connection_type}',
                'connection_type': connection_type
            })
        else:
            return jsonify({
                'success': False,
                'error': 'No se pudo establecer la conexión'
            }), 400
            
    except Exception as e:
        logger.error(f"Error en /api/connect: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/disconnect', methods=['POST'])
def disconnect():
    """Endpoint para desconectar del dron"""
    try:
        success = mavlink_conn.disconnect()
        
        if success:
            return jsonify({
                'success': True,
                'message': 'Desconectado exitosamente'
            })
        else:
            return jsonify({
                'success': False,
                'error': 'Error al desconectar'
            }), 400
            
    except Exception as e:
        logger.error(f"Error en /api/disconnect: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/telemetry', methods=['GET'])
def get_telemetry():
    """Endpoint para obtener telemetría"""
    try:
        if not mavlink_conn.connected:
            return jsonify({
                'success': False,
                'error': 'No conectado'
            }), 400
        
        telemetry_data = mavlink_conn.get_telemetry()
        
        return jsonify({
            'success': True,
            'data': telemetry_data,
            'timestamp': time.time()
        })
        
    except Exception as e:
        logger.error(f"Error en /api/telemetry: {e}")
        return jsonify({
            'success': False,
            'error': str(e)
        }), 500

@app.route('/api/status', methods=['GET'])
def get_status():
    """Endpoint para obtener estado de conexión"""
    try:
        return jsonify({
            'connected': mavlink_conn.connected,
            'connection_type': mavlink_conn.connection_type,
            'simulate_mode': mavlink_conn.simulate_mode,
            'last_update': mavlink_conn.last_telemetry.get('timestamp', None)
        })
        
    except Exception as e:
        logger.error(f"Error en /api/status: {e}")
        return jsonify({
            'connected': False,
            'error': str(e)
        }), 500

@app.errorhandler(404)
def not_found(error):
    return jsonify({'error': 'Endpoint no encontrado'}), 404

@app.errorhandler(500)
def internal_error(error):
    return jsonify({'error': 'Error interno del servidor'}), 500

def cleanup():
    """Limpieza al cerrar la aplicación"""
    logger.info("Cerrando aplicación...")
    mavlink_conn.disconnect()

if __name__ == '__main__':
    try:
        logger.info("🚁 Iniciando servidor MAVLink Flask")
        logger.info("📍 Modo simulación disponible con coordenadas de A Coruña")
        logger.info("🌐 Servidor ejecutándose en http://localhost:5000")
        
        # Registrar función de limpieza
        import atexit
        atexit.register(cleanup)
        
        # Ejecutar servidor
        app.run(
            host='10.207.0.112',
            port=3001,
            debug=True,
            threaded=True,
            use_reloader=False  # Evitar problemas con threads
        )
        
    except KeyboardInterrupt:
        logger.info("Servidor detenido por el usuario")
        cleanup()
    except Exception as e:
        logger.error(f"Error fatal: {e}")
        cleanup()
