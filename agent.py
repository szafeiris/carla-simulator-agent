import os, sys, time, random, logging, glob, datetime
import math
import numpy as np
import cv2

CARLA_PATH = '../'
try:
    sys.path.append(glob.glob(CARLA_PATH + '../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

## 
## Constant declaration
##
SERVER       = "localhost"
PORT         = 2000
IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480
FOV          = 120


logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO, handlers=[
        logging.FileHandler('logs/' + datetime.datetime.now().strftime("%d-%m-%Y %H-%M-%S") + '.log'),
        logging.StreamHandler()
    ])

def image_processing(image): 
    # image.save_to_disk('imgdata/%08d' % image.frame_number) # Saves image to disk
    
    # Variable image.raw_data is an array of BGRA 32-bit pixels
    rgb_img = np.array(image.raw_data).reshape((IMAGE_WIDTH, IMAGE_HEIGHT, 4))[:,:,:3]

    cv2.imshow("RGB Camera output", rgb_img)
    cv2.waitKey(1)
    
    return rgb_img/255


def main():
    actor_list = []

    # Connect to server
    logging.info(f"Connecting to {SERVER}:{PORT}...")
    client = carla.Client(SERVER, PORT)
    client.set_timeout(2.0)
    logging.info("Connected to server!")


    try:
        logging.info("Retrieving world data...")
        world = client.get_world()
        
        logging.info("Loading blueprint library...")
        blueprints = world.get_blueprint_library()

        logging.info("Loading test vehicle...")
        vehicle_blueprint = blueprints.find('vehicle.ford.mustang')                     # Vehicle blueprint
        vehicle_blueprint.set_attribute('color', '255,0,0')                             # Red Mustang!

        vehicle = world.spawn_actor( vehicle_blueprint,      
                                     random.choice(world.get_map().get_spawn_points())  # Spawn point
                                   )
        actor_list.append(vehicle)                                                      # Add vehicle to actor list

        vehicle.apply_control(carla.VehicleControl(throttle=1.0, steer=0.0))            # Start moving forward

        logging.info("Loading vehicle RGB camera sensor...")
        rgb_cam_blueprint = blueprints.find('sensor.camera.rgb')
        rgb_cam_blueprint.set_attribute('image_size_x', f'{IMAGE_WIDTH}')
        rgb_cam_blueprint.set_attribute('image_size_y', f'{IMAGE_HEIGHT}')
        rgb_cam_blueprint.set_attribute('fov', f'{FOV}')

        rgb_cam = world.spawn_actor( rgb_cam_blueprint, 
                                     carla.Transform(carla.Location(x=2.4, z=0.5)),      # Ajust sensor location on vechicle
                                     attach_to=vehicle                                   # Attach RGB cammera to test vehicle
                                   )
        actor_list.append(rgb_cam)                                                       # Add camera to actor list

        rgb_cam.listen(lambda img_data: image_processing(img_data))                      # Process every frame

        logging.info("Simulation started.")
        time.sleep(5)
        while True:
            vel = vehicle.get_velocity()
            v = int(3.6 * math.sqrt(vel.x**2 + vel.y**2 + vel.z**2))
            print("Speed: " + str(v))
            pass

    except KeyboardInterrupt:
        logging.warning("KeyboardInterrupt exception caught! Terminating session...")
    except RuntimeError as re:
        if 'time-out of 2000ms while waiting for the simulator, make sure the simulator is ready and connected to' in str(re):
            logging.error("Could not establish connection with the selected server.")

    finally:
        logging.info("Destroying actors...")
        for actor in actor_list:
            actor.destroy()
        logging.info("Execution finished!")

if __name__ == '__main__':
    main()