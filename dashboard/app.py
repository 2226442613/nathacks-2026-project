"""Local Flask management API. No physical-control routes or imports."""
from flask import Flask,jsonify,render_template,request,Response

from .service import DashboardService
from .model import STATUSES


def create_app(service=None):
    app=Flask(__name__,static_folder='static',template_folder='templates')
    service=service or DashboardService()
    app.extensions['dashboard']=service
    app.config['MAX_CONTENT_LENGTH']=8192

    @app.before_request
    def local_mutations():
        if request.method=='POST':
            if not request.is_json:return jsonify(error='Use application/json'),415
            origin=request.headers.get('Origin')
            if origin and origin!=request.host_url.rstrip('/'):return jsonify(error='Same-origin requests required'),403

    @app.after_request
    def headers(response):
        response.headers['Cache-Control']='no-store' if request.path.startswith('/api/') else 'no-cache'
        response.headers['X-Content-Type-Options']='nosniff'
        response.headers['Content-Security-Policy']="default-src 'self'; script-src 'self'; style-src 'self'; img-src 'self' data:; object-src 'none'; frame-ancestors 'none'"
        return response

    @app.errorhandler(ValueError)
    def invalid(exc):return jsonify(error=str(exc)),400
    @app.errorhandler(KeyError)
    def missing(exc):return jsonify(error='Task not found'),404
    @app.errorhandler(RuntimeError)
    def conflict(exc):return jsonify(error=str(exc)),409

    @app.get('/')
    def index():return render_template('index.html')

    @app.get('/health')
    def health():return jsonify(status='ok',non_actuating=True)

    @app.get('/api/state')
    def state():return jsonify(service.snapshot(request.args.get('status') or None,request.args.get('priority') or None))

    @app.post('/api/mode')
    def mode():
        service.set_mode(request.get_json().get('mode'))
        return jsonify(service.snapshot())

    @app.post('/api/demo/<action>')
    def demo(action):
        service.demo_control(action);return jsonify(service.snapshot())

    @app.post('/api/monitor/retry')
    def retry():
        service.retry_live();return jsonify(service.snapshot())

    @app.get('/api/tasks/<path:key>')
    def task(key):
        found=next((t for t in service.snapshot()['scene_targets'] if t['task_id']==key),None)
        if found is None:raise KeyError(key)
        return jsonify(found)

    @app.post('/api/tasks/<path:key>/status')
    def transition(key):
        body=request.get_json()
        if not isinstance(body,dict) or set(body)!={'status'}:raise ValueError('Only a task status is accepted; verification cannot be asserted')
        service.transition(key,body['status'])
        return jsonify(service.snapshot())

    @app.get('/api/frame.jpg')
    def frame():
        with service.lock:
            if service.mode!='LIVE' or not service.jpeg:return jsonify(error='Live camera image unavailable'),503
            return Response(service.jpeg,mimetype='image/jpeg')

    @app.get('/api/stream.mjpg')
    def stream():
        with service.lock:
            if service.mode!='LIVE' or not service.jpeg:return jsonify(error='Live camera image unavailable'),503
        return Response(service.frame_stream(),content_type='multipart/x-mixed-replace; boundary=frame')

    return app
