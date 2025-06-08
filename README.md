Monitoreo-MAVLink es un proyecto que proporciona una interfaz web para monitorear y controlar vehículos aéreos no tripulados (UAV) que utilizan el protocolo MAVLink. El proyecto incluye varios archivos, como app.py, index.html, mavlink_relay_full.py, script.js y style.css.

Puntos Principales de Funcionalidad
Proporcionar una interfaz web para monitorear y controlar UAVs que utilizan el protocolo MAVLink
Permitir la visualización de datos en tiempo real, como posición, altitud, velocidad y estado del vehículo

Lenguajes usados:
Python
HTML
CSS
JavaScript

La aplicación para levantar los servidores TCP y UDP deben de estar en otra terminal para poder ejecutar la pagina y este al mismo tiempo.

├── mavlink_relay_full
mavlink_web/
├── app.py              # Backend Flask  
├── templates/
│   └── index.html     # HTML limpio
└── static/
    ├── css/
    │   └── style.css  # Estilos separados
    └── js/
        └── script.js  # JavaScript modular
