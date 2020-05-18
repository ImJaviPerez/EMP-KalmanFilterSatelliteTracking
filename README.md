# EMP-Satellite-Tracking-using-Arduino
Using a Kalman Filter for Satellite Tracking. Arduino implementation. 
End of Master's Project. Basque Country University. https://www.ehu.eus/es/web/master/master-modelizacion-investigacion-matematica-estadistica-computacion

This master thesis implements a model for predicting amateur radio satellites attitude. This archetype runs in an \href{https://www.arduino.cc/}{Arduino}(\url{https://www.arduino.cc/}) low cost hardware and free software. Those MPU have small memory and CPU resources, so it is necessary to choose an accurate and quick algorithm but not too weighty to be able to be executed in those equipment. The selected algorithm will use Kepler-Newton dynamics pricipia and a Kalman filter.

This Kalman filter runs with an accelerometer, a gyroscope and a digittal compass.
