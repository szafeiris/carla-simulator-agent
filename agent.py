import os, sys, time, random, logging, glob, datetime
import math, pickle
import numpy as np
import cv2

logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO, handlers=[
        logging.FileHandler('logs/' + datetime.datetime.now().strftime("%d-%m-%Y %H-%M-%S") + '.log'),
        logging.StreamHandler()
    ])

random.seed(1)

CARLA_PATH = '../'
logging.info("Importing carla...")
try:
    sys.path.append(glob.glob(CARLA_PATH + '../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    logging.error("Could not find carla packages!")
import carla


## 
## Constant declaration
##

SERVER       = "localhost"
PORT         = 2000

IMAGE_WIDTH  = 640
IMAGE_HEIGHT = 480
FOV          = 120

SAVE_IMAGE   = True
SHOW_IMAGE   = True

IMAGE_FOLDER = "img_data/"
DATA_FL_NAME = "data.bin"

CAR_SPAWN_NO = 9

HISTORY_LIM  = 4000

ACT_THROTTLE = 0
ACT_BRAKE    = 1
ACT_LEFT     = 2
ACT_RIGHT    = 3
ACT_NOTHNG   = 4

PNLT_COLLIS  = 50
PNLT_HGHSPD  = 20
PNLT_LOWSPD  = 10

INC_NEEDED   = 10000

AVLBL_ACTNS  = [ACT_THROTTLE, ACT_BRAKE, ACT_LEFT, ACT_RIGHT, ACT_NOTHNG]

# Function used from carla manual_driving.py code
def get_actor_display_name(actor, truncate=250):
    name = ' '.join(actor.type_id.replace('_', '.').title().split('.')[1:])
    return (name[:truncate - 1] + u'\u2026') if len(name) > truncate else name


class Car(object):

    def __init__(self, world):
        self.actors = []

        car_blueprint  = random.choice(world.get_blueprint_library().filter('vehicle.*'))   # Get available cars
        spawn_point    = random.choice(world.get_map().get_spawn_points())                  # Get available spawn points

        vehicle        = world.try_spawn_actor(car_blueprint, spawn_point)                  # Try to spawn a vahicle
        while vehicle is None:                                                              # try_spawn_actor will return None on failure
            spawn_point    = random.choice(world.get_map().get_spawn_points())
            vehicle        = world.try_spawn_actor(car_blueprint, spawn_point)
       
        vehicle.set_autopilot(True)                                                         # This is a simple car used for testing
        self.actors.append(vehicle)                                                         # collision avoidance of out agent

    def destroy_actors(self):
        for actor in self.actors:
            actor.destroy()
    
    def reset(self, world):
        self.__init__(world)


class CarAgent(Car):

    def image_processing(self, image):
        if SAVE_IMAGE:
            image.save_to_disk( IMAGE_FOLDER + '%08d' % image.frame_number)                 # Save image to disk (if enabled)

        # Variable image.raw_data is an array of BGRA 32-bit pixels
        rgb_img = np.array(image.raw_data).reshape((IMAGE_WIDTH, IMAGE_HEIGHT, 4))[:,:,:3]  # Transform image to RGB format

        if SHOW_IMAGE:
            cv2.imshow("RGB Camera output", rgb_img)
            cv2.waitKey(1)
    
        self.rgb_img = rgb_img

    def on_collision(self, event):
        actor_type = get_actor_display_name(event.other_actor)
        logging.info('Collision with %r' % actor_type)
        
        impulse = event.normal_impulse                                                      # Calculate impact made from collision
        intensity = math.sqrt(impulse.x**2 + impulse.y**2 + impulse.z**2)
        self.history.append((event.frame_number, intensity))
        if len(self.history) > HISTORY_LIM:
            self.history.pop(0)


    def __init__(self, world):
        self.actors = []
        self.history = []
        self.simulate = True

        car_blueprint  = world.get_blueprint_library().find('vehicle.ford.mustang')         # Get our test car
        car_blueprint.set_attribute('color', '255,0,0')                                     # Red Mustang!

        spawn_point    = random.choice(world.get_map().get_spawn_points())                  # Get available spawn points

        vehicle        = world.try_spawn_actor(car_blueprint, spawn_point)                  # Try to spawn the vehicle
        while vehicle is None:                                                              # try_spawn_actor will return None on failure
            spawn_point    = random.choice(world.get_map().get_spawn_points())
            vehicle        = world.try_spawn_actor(car_blueprint, spawn_point)
       
        #vehicle.set_autopilot(True)
        self.actors.append(vehicle)
        self.vehicle = vehicle


        ## Add a camera to the vehicle
        rgb_cam_blueprint = world.get_blueprint_library().find('sensor.camera.rgb')
        rgb_cam_blueprint.set_attribute('image_size_x', f'{IMAGE_WIDTH}')
        rgb_cam_blueprint.set_attribute('image_size_y', f'{IMAGE_HEIGHT}')
        rgb_cam_blueprint.set_attribute('fov', f'{FOV}')

        rgb_cam = world.spawn_actor( rgb_cam_blueprint, 
                                     carla.Transform(carla.Location(x=2.4, z=0.5)),      # Ajust sensor location on vechicle
                                     attach_to=self.vehicle)                             # Attach RGB cammera to test vehicle
        self.actors.append(rgb_cam)                                                      # Add camera to actor list
        self.rgb_cam = rgb_cam
        rgb_cam.listen(lambda img_data: self.image_processing(img_data))                 # Process every frame


        ## Add a collision sensor to the vehicle
        sensor = world.spawn_actor(world.get_blueprint_library().find('sensor.other.collision'), 
                                   carla.Transform(), 
                                   attach_to=self.vehicle)
        self.actors.append(sensor)                                                       # Add camera to actor list
        self.sensor = sensor
        sensor.listen(lambda event: self.on_collision(event))


    def apply_vehicle_control(self, action):
        if action == ACT_THROTTLE:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer=0.0))
        elif action == ACT_BRAKE:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
        elif action == ACT_LEFT:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer=-1.0))
        elif action == ACT_RIGHT:
            self.vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer=1.0))
        elif action == ACT_NOTHNG:
            self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0))
        else:
            logging.error(['Action ', str(action), ' is not valid!'])

    def get_applied_vehicle_control(self):
        control = self.vehicle.get_control()
        return (control.throttle, control.brake, control.steer)
    
    def get_last_img(self):
        return self.rgb_img
    
    def calculate_vehicle_velocity(self):
        velocity = self.vehicle.get_velocity()
        return int(3.6 * math.sqrt(velocity.x**2 + velocity.y**2 + velocity.z**2))       # Kilometers per hour
    
    def enable_autopilot(self):
        self.vehicle.set_autopilot(True)
    
    def disable_autopilot(self):
        self.vehicle.set_autopilot(False)

    def get_current_reward(self):
        v = self.calculate_vehicle_velocity()
        reward = 0

        if v < 40:
            reward = reward - PNLT_LOWSPD
        elif v > 80:
            reward = reward - PNLT_HGHSPD
        elif len(self.history) != 0:
            reward = reward - PNLT_COLLIS
            self.simulate = False
        else:
            reward = reward + 10

        return (reward, self.simulate)

    
    def step(self, action):
        self.apply_vehicle_control(action)
        (r, s) = self.get_current_reward()

        return (action, r, s)

    def reset(self, world):
        self.__init__(world)
        self.vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=0.0, steer=0.0))
        self.history.clear()


class Enviroment():
    def __init__(self):
        self.cars = []
        self.world = None
        self.incidents = []

        # Connect to server
        logging.info(f"Connecting to {SERVER}:{PORT}...")
        client = carla.Client(SERVER, PORT)
        client.set_timeout(2.0)
        logging.info("Connected to server!")

        try:
            logging.info("Retrieving world data...")
            self.world = client.get_world()
        except KeyboardInterrupt:
            logging.error("KeyboardInterrupt exception caught! Terminating session...")
        except RuntimeError as re:
            if 'time-out of 2000ms while waiting for the simulator, make sure the simulator is ready and connected to' in str(re):
                logging.error("Could not establish connection with the selected server.")
        finally:
            if self.world is None:
                logging.critical('Could not initiate comunication with server.')
                exit(1)

        for i in range(CAR_SPAWN_NO):                                       # Generate random cars
            self.cars.append(Car(self.world))                                    
        self.carAgent = CarAgent(self.world)                                # Create car agent
        self.cars.append(self.carAgent)

        
    def reset(self):       
        for a in self.cars:
            a.reset(self.world)

    def run_gather(self):
        self.reset()
        batch_size = int(0.25 * INC_NEEDED)
        #batches    = int(INC_NEEDED/batch_size)
        batches    = 1

        try:
            self.carAgent.enable_autopilot()                                # Drive autonomusly to create a dataset
            logging.info("Simulation started.")
            time.sleep(5)
            while batches > 0:
                while len(self.incidents) < INC_NEEDED:
                    self.incidents.append( (self.carAgent.get_applied_vehicle_control(), self.carAgent.get_current_reward(), self.carAgent.get_last_img()) )
                batches = batches - 1

                # Flush incidents to file
                #file = open(DATA_FL_NAME, 'a')
                #for d in self.incidents:
                #    pickle.dump(d, file)
                #self.incidents.clear()

            logging.info("Simulation finished.")
            
        except KeyboardInterrupt:
            logging.warning("KeyboardInterrupt exception caught! Terminating session...")
        except RuntimeError as re:
            if 'time-out of 2000ms while waiting for the simulator, make sure the simulator is ready and connected to' in str(re):
                logging.error("Could not establish connection with the selected server.")
        finally:
            for car in self.cars:
                car.destroy_actors()
            logging.info("Simulation exited.")

    def get_incidents(self):
        return self.incidents


def main():
    env = Enviroment()

    try:
        env.run_gather()
    except Exception as e:
        print(e)
    finally:
        for ecar in env.cars:
            ecar.destroy_actors()


    inc = env.get_incidents()[80]
    print(inc[0])
    print(inc[1])
    print(inc[2].shape)
    print(len(env.get_incidents()))

    cv2.imshow("Final", inc[2])
    x = input("Press enter to exit...")



if __name__ == '__main__':
    main()