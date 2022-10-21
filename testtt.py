import requests

while True:
    s = requests.get('http://localhost:8081/SPR?data={"ReferenceMarker":"1550100","EndEffectorMarker":"66600","Method":"With","RobotPose":[-332.365, 395.307, 188.658, -160.751, 10.469, -78.64]}')
    print(s.json())