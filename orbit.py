import matplotlib.pyplot as plt
import numpy as np
drone1 = np.array([5.,0.])
drone2 = np.array([3.,3.])
R = 5
offset = 1
speed = .5
maxSpeed = 1
phase = 0
def iter(d1, d2):
	global phase
	v1 = np.zeros(2)
	v2 = np.zeros(2)
	avoid_mag = 3/max(np.linalg.norm(d2-d1)-offset,1e-5)**2
	v1 += (d1-d2)*avoid_mag
	v2 += (d2-d1)*avoid_mag
	target = np.array([R*np.cos(phase),R*np.sin(phase)])
	dist = np.linalg.norm(target-d1)
	if dist>1e-5:
		v1+=dist*((target-d1)/dist)
	v1norm = np.linalg.norm(v1)
	v1/=v1norm
	v1*=min(maxSpeed, v1norm)
	d1+=v1
	print(v1)
	phase += 0.05
	# d2+=v2
	return target

	
# reset plt
for i in range(100):
	target = iter(drone1,drone2)
	plt.clf()
	figure, axes = plt.subplots()
	drone1_circle = plt.Circle(drone1, 1)
	drone2_circle = plt.Circle(drone2, 1)
	target_circle = plt.Circle(target, .3, color='r')
	
	axes.set_aspect( 1 )
	axes.add_artist( drone1_circle )
	axes.add_artist( drone2_circle )
	axes.add_artist( target_circle )
	plt.xlim([-10,10])
	plt.ylim([-10,10])
	plt.savefig('imgs/orbit'+str(i)+'.png')
	plt.close()
print("done")

from PIL import Image
import glob
 
# Create the frames
frames = []
imgs = glob.glob("imgs/*.png")
imgs.sort(key = lambda x: int(x.split('.')[0].split('orbit')[-1]))
for i in imgs:
    new_frame = Image.open(i)
    frames.append(new_frame)
 
# Save into a GIF file that loops forever
frames[0].save('png_to_gif.gif', format='GIF',
               append_images=frames[1:],
               save_all=True,
               duration=50, loop=0)