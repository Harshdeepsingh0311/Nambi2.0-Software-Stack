
from camera import CameraStream
import cv2
#app = Flask(__name__)

#cap = CameraStream("rtsp://admin:ABCdef123%@192.168.1.251:554/cam/realmonitor?channel=1&subtype=0").start()
#cap2 = CameraStream().start()

def rescale_frame(frame, percent=75):
    width = int(frame.shape[1] * percent/ 100)
    height = int(frame.shape[0] * percent/ 100)
    dim = (width, height)
    return cv2.resize(frame, dim, interpolation =cv2.INTER_AREA)
'''
@app.route('/')
def index():
    """Video streaming home page."""
    return render_template('index.html')


def gen_frame():
    """Video streaming generator function."""
    while cap:
        frame = cap.read()
        convert = cv2.imencode('.jpg', frame)[1].tobytes()
        yield (b'--frame\r\n'
               b'Content-Type: image/jpeg\r\n\r\n' + convert + b'\r\n') # concate frame one by one and show result


@app.route('/video_feed')
def video_feed():
    """Video streaming route. Put this in the src attribute of an img tag."""
    return Response(gen_frame(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

'''
if __name__ == '__main__':
    #cap = CameraStream("rtsp://admin:Teaminferno@192.168.1.250:554/cam/realmonitor?channel=1&subtype=0").start()
    cap1 = CameraStream("rtsp://admin:ABCdef123@192.168.1.64:554/cam/realmonitor?channel=1&subtype=0").start()
    #cap2 = CameraStream("rtsp://admin:ABCdef123%@192.168.1.251:554/cam/realmonitor?channel=1&subtype=0").start()
    while True :
        #frame = cap.read()
        frame1 = cap1.read()
        #frame2 = cap2.read()
        #frame75 = rescale_frame(frame, percent=60) 
        frame76 = rescale_frame(frame1, percent=60) 
        #frame77 = rescale_frame(frame2, percent=60) 
        #cv2.imshow('CAM1', frame75)
        cv2.imshow('CAM2', frame76)
        #cv2.imshow('CAM3', frame77)
        if cv2.waitKey(1) == 27 :
            break
    cap.stop()
    cv2.destroyAllWindows()   

