def add_dashboard_command(commands):
    parser=commands.add_parser('dashboard',help='Hospital cleaning management demo and non-actuating live monitoring')
    parser.add_argument('--host',default='127.0.0.1')
    parser.add_argument('--port',type=int,default=5050)
    parser.add_argument('--mode',choices=['demo','live'],default='demo')
    parser.add_argument('--calibration',default='config/camera_config.json')
    parser.add_argument('--hardware',default='config/cleaning_hardware.json')
    parser.add_argument('--camera',type=int,default=0)
    parser.add_argument('--width',type=int,default=1280);parser.add_argument('--height',type=int,default=720)
    parser.add_argument('--background',default='data/iphone_reference.png')
    parser.add_argument('--settings',default='config/spill_config.json')
    parser.add_argument('--localization-settings',default='config/localization_config.json')
    parser.add_argument('--robot-settings',default='config/robot_config.json')
    parser.add_argument('--marker-plane',default='config/robot_marker_plane.json')
    parser.add_argument('--navigation-events',help='Optional Module06 progress.jsonl; display-only, never starts robot control')
    parser.set_defaults(headless=True)


def run_dashboard(args):
    # Registering original CLI commands must not load the optional web runtime.
    from werkzeug.serving import make_server
    from .app import create_app
    from .service import DashboardService

    if args.host not in {'127.0.0.1','localhost','::1'}:raise ValueError('Dashboard is a local demonstration; bind to localhost')
    service=DashboardService(args.calibration,hardware=args.hardware,navigation_events=args.navigation_events,live_options={
        'camera':args.camera,'width':args.width,'height':args.height,'background':args.background,
        'settings':args.settings,'localization_settings':args.localization_settings,
        'robot_settings':args.robot_settings,'marker_plane':args.marker_plane})
    app=create_app(service)
    server=make_server(args.host,args.port,app,threaded=True)
    try:
        service.start()
        if args.mode=='live':service.set_mode('LIVE')
        print(f'Hospital Guardian Dashboard: http://{args.host}:{server.server_port} | {args.mode.upper()} | no hardware actuation',flush=True)
        server.serve_forever()
    finally:
        try:service.close()
        finally:server.server_close()
