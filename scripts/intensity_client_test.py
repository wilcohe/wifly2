import sys, os
import rospy, rospkg
from wifly2.srv import *
from geometry_msgs.msg import Point
from nav_msgs.msg import OccupancyGrid
import numpy as np

def intensity_query_client(pos): 
	rospy.wait_for_service("intensity")
	try: 
		intensity_serv = rospy.ServiceProxy("intensity", intensity)
		intensity_val = intensity_serv(pos)
		return intensity_val
	except rospy.ServiceException as e: 
		print(f"Exception {e} occurred")

def gen_wifi(width, height, res):

	occ_map = OccupancyGrid()
	occ_map.info.resolution = res
	occ_map.info.width = int(width)
	occ_map.info.height = int(height)
	occ_map.info.origin.position.x = 0
	occ_map.info.origin.position.y = 0

	# Map is 368 x 275 currently
	size = np.array([width*res, height*res])
	bounds = np.array([width, height])
	rays = np.arange(0, 2*np.pi, 0.005)
	w_map = np.zeros((int(height), int(width)))
	router = np.array(size)/2
	step = 0.001
	pt = Point()

	for ray in rays: 

		if ray % 0.25 == 0: 
			print("50 its")

		pos = np.copy(router)
		wifi = 1
		d_o_m = np.array([np.cos(ray), np.sin(ray)])

		while all(pos > 0) and all(pos < size) and wifi > 0.1:

			pos_pt = gen_pt(pos)

			occ = intensity_query_client(gen_pt(pos))

			if occ.intensity > 0:
				wifi *= 0.9
				pos += 0.04*d_o_m
			elif all(np.array([occ.x, occ.y]) < bounds):

				if w_map[occ.y, occ.x] == 0: 
					w_map[occ.y, occ.x] = int(round((1 - wifi)*-100))

				else:
					w_map[occ.y, occ.x] += round((1 - wifi)*-100)
					w_map[occ.y, occ.x] = int(w_map[occ.y, occ.x]/2)

				wifi *= 0.995

				pos += 0.04*d_o_m
			else:
				break

	occ_map_flat = w_map.ravel().astype(int)
	np.savetxt("wifi_map.txt", occ_map_flat)
	occ_map.data = occ_map_flat.tolist()



	return occ_map




def gen_pt(pos):
	pt = Point()
	pt.x = pos[0]
	pt.y = pos[1]
	return pt


if __name__ == "__main__": 

	w = float(sys.argv[1])
	h = float(sys.argv[2])
	r = float(sys.argv[3])

	_PATH = os.path.join(rospkg.RosPack().get_path("wifly2"), "config/maps/wifi_map.txt")

	if os.path.exists(_PATH): 
		print("Loading WiFi data from source. ")
		rospy.loginfo("Running with %s", _PATH)
		data = np.loadtxt(_PATH)
		wifi_sig = OccupancyGrid()
		wifi_sig.info.resolution = r
		wifi_sig.info.width = int(w)
		wifi_sig.info.height = int(h)
		wifi_sig.info.origin.position.x = 0
		wifi_sig.info.origin.position.y = 0
		wifi_sig.data = data.astype(int).tolist()

	else:
		wifi_sig = gen_wifi(w, h, r)

		print("Finish generating wifi signal. ")

	map_pub = rospy.Publisher("/wifi_map", OccupancyGrid, queue_size=1, latch=True)
	rospy.init_node("wifi_data")

	rospy.sleep(3)

	print("Publishing Wifi")

	map_pub.publish(wifi_sig)


	rospy.spin()

	print(val)
