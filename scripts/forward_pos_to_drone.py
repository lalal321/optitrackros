#!/usr/bin/env python
import natnetclient as natnet
# import dronekit
import argparse
import sys
import rospy
from optitrackros.msg import Camera
import numpy as np

if __name__ == "__main__":
# Command line arguments
  parser = argparse.ArgumentParser(description='Simple program that forwards object position to drone.')
  parser.add_argument('--natnet_ip', dest='natnet_ip', action='store', type=str, default='192.168.0.100',help='IP of the machine running Optitrack Motive (default: 192.168.0.100)')
  # parser.add_argument('--mav_link', dest='mav_link', action='store', type=str, default='udp:127.0.0.1:14551',
  # help='Interface to the drone (default: udp:127.0.0.1:14551)')
  parser.add_argument('--body_name', dest='body_name', action='store', type=str, default="Plane",help='The body name of the optitrack rigid body, if not inputted, will send first key')
  parser.add_argument('-v', "--verbose", action='store_true', help='Turns on verbose messages.')
  args = parser.parse_args()
  # Connect to Motive
  try:
    client = natnet.NatClient(client_ip=args.natnet_ip,data_port=1511,comm_port=1510)
  except Exception as e:
    print("Could not connect to NatNet computer at IP " + args.natnet_ip)
    print(e)
    sys.exit(0)
    # Connect to the drone
    # vehicle = dronekit.connect(args.mav_link, wait_ready=False, rate=200, heartbeat_timeout=0)
    #print client.rigid_bodies
    # embed()
  ## Create the ros node
  rospy.init_node('CameraNode', anonymous=True)
  rate = rospy.Rate(100) # run the loop at these many Hz
  pub = rospy.Publisher('CameraStream',Camera,queue_size = 10);
  # pub_plane = rospy.Publisher('CameraStream_Plane', Camera, queue_size = 10)
  # pub_pole = rospy.Publisher('CameraStream_Pole',  Camera, queue_size = 10)
  msg = Camera();
  bodynames = {'Origin':100,'LPS_x':101,'LPS_y':106,'Car':104,'Plane':105,'Center':107};
  while not rospy.is_shutdown():
    # body_name = args.body_name
    for body_name in bodynames.keys():
    	if body_name in  client.rigid_bodies:
    		body = client.rigid_bodies[body_name]
    		if body.seen == True:
				qw = body.rotation[3]       # transformations from camera to LPS Frame:(y -> z axis , z-> -y),
				qroll = body.rotation[0]
				qpitch = body.rotation[1]
				qyaw = body.rotation[2]
				if args.verbose:
				  print 'Position: (' + str(body.position.x) + ', ' + str(-body.position.y) + ', ' + str(-body.position.z) + ')'
				  print 'Rotation: (' + str(qw) + ', ' + str(qroll) + ', ' + str(qpitch) + ', ' + str(qyaw) + ')'
				  print 'Timestamp: (' + str(client.timestamp) + ')'
				  print ''
				msg.label = bodynames[body_name];
				msg.pos = np.array([body.position.x,body.position.y,body.position.z]);
				msg.quart = np.array([qw,qroll,qpitch,qyaw]);
				msg.timestamp = client.timestamp;
				pub.publish(msg);
        # if body_name == 'Plane':
        #     pub_plane.publish(msg)
        # if body_name == 'LPSOrigin':
        #     pub_pole.publish(msg)
    rate.sleep()
          # Prepare message for drone
          # msg = vehicle.message_factory.att_pos_mocap_encode(time_usec=time.time(),
          #                                                    q=[qw, qroll, qpitch, qyaw],
          #                                                    x=body.position.x,
          #                                                    y=-body.position.y,
          #                                                    z=-body.position.z)

          # Send message
      #     vehicle.send_mavlink(msg)

      # # notify connexion LOSS
      # if vehicle.last_heartbeat > 5:
      #     print("Connexion lost")
