# Little_Satellite_Model

It is a little satellite project which consist of:
  -Ground Station(node00)
  -Satellite(node01)
  -Remote Car(node02)
  
 The sensors used in this project:
  -Air quality sensor
  -Pressure sensor
  -Temperature Sensor
 Nrf24l01 tranciever module used for communication.
 There is an interface designed on Java, processing tool which seems like:
 ![interface](https://user-images.githubusercontent.com/43203464/97483377-65ca9300-1968-11eb-9a10-9083771bb658.jpeg)
 
 You can see the telemetry values of satellite and the car on screen and control the car with W,A,S,D keys.
 
 Working Principle:
  There is a two way communication between 3 nodes: Ground station sends data to satellite and satellite sends data to car. Then reverse of this process realized.
  
 
