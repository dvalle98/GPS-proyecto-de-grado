========================================= VERSION ESTABLE ==========================================

He optimizado el código para garantizar su funcionamiento continuo 24/7 mediante los siguientes cambios:

-Reducción del uso de variables globales: Se ha minimizado el uso de variables globales para liberar memoria y optimizar el uso de recursos. Esto mejora la eficiencia del sistema al limitar el espacio en memoria asignado para almacenar datos que no son necesarios globalmente. Además, al limitar el alcance de las variables a las funciones o bloques de código donde se necesitan, se mejora la modularidad del código y se facilita su comprensión y mantenimiento, reduciendo el riesgo de conflictos de nombres y errores difíciles de depurar.

-Uso de buffers en lugar de objetos String: Se han reemplazado los objetos String por buffers para gestionar eficientemente la memoria y evitar problemas de fragmentación. Esto mejora la estabilidad del sistema y reduce el riesgo de agotamiento de recursos.

Estos cambios aseguran que el código funcione de manera óptima y estable, lo que permite su despliegue en entornos de funcionamiento continuo 24/7 sin riesgo de interrupciones o fallos inesperados.