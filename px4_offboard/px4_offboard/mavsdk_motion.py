import asyncio
import rclpy
from rclpy.node import Node
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionGlobalYaw, PositionNedYaw

landing_zone_null = PositionGlobalYaw(48.506769, -71.646686, 30, 0, PositionGlobalYaw.AltitudeType.REL_HOME)
landing_zone_c = PositionGlobalYaw(48.506712, -71.64691, 30, 0, PositionGlobalYaw.AltitudeType.REL_HOME)
landing_zone_b = PositionGlobalYaw(48.506816, -71.64684, 30, 0, PositionGlobalYaw.AltitudeType.REL_HOME)
landing_zone_a = PositionGlobalYaw(48.506772, -71.646903, 30, 0, PositionGlobalYaw.AltitudeType.REL_HOME)

selected_landing_zone = landing_zone_null
search_radius = 3

class MavsdkMotion(Node):

    def __init__(self):
        super().__init__('mavsdk_motion')

        self.drone = System()
        
        asyncio.get_event_loop().run_until_complete(self.drone.connect())

        self.print_mission_progress_task = asyncio.ensure_future(
            self.print_mission_progress()
        )

        self.running_tasks = [self.print_mission_progress_task]

        self.termination_task = asyncio.ensure_future(
            self.observe_is_in_air(self.running_tasks)
        )

        asyncio.get_event_loop().run_until_complete(self.run())
               

    async def connect(self):
        await self.drone.connect()
        self.get_logger().info("Waiting for drone to connect...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                self.get_logger().info("-- Connected to drone!")
                break

    async def import_and_upload_mission(self, mission_file):
        mission_import_data = await self.drone.mission_raw.import_qgroundcontrol_mission(mission_file)
        self.get_logger().info(f"{len(mission_import_data.mission_items)} mission items imported")
        await self.drone.mission_raw.upload_mission(mission_import_data.mission_items)
        self.get_logger().info("Mission uploaded")

    async def flight_check(self):
        self.get_logger().info("Waiting for drone to have a global position estimate...")
        async for health in self.drone.telemetry.health():
            if health.is_global_position_ok and health.is_home_position_ok:
                self.get_logger().info("-- Global position estimate OK")
                break
            else:
                await asyncio.sleep(1)
                await self.flight_check()

    async def arm(self):
        self.get_logger().info("-- Arming")
        await self.drone.action.arm()

    async def start_mission(self):
        self.get_logger().info("-- Starting mission")
        await self.drone.mission_raw.start_mission()

    async def print_mission_progress(self):
        async for mission_progress in self.drone.mission_raw.mission_progress():
            self.get_logger().info(
                f"Mission progress: {mission_progress.current}/{mission_progress.total}"
            )

            # Check if the mission is completed
            if mission_progress.current == mission_progress.total:
                async for flight_mode in self.drone.telemetry.flight_mode():
                    if str(flight_mode) == "HOLD":
                        # Start offboard mode
                        self.get_logger().info("Drone is in hold mode and mission is completed.")

                        self.get_logger().info("-- Starting offboard")
                        try:
                            await self.drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
                            await self.drone.offboard.start()
                            await self.go_to_search_area(selected_landing_zone)
                            await self.drone.offboard.stop()
                        except OffboardError as error:
                            self.get_logger().error(
                                f"Starting offboard mode failed with error code: {error._result.result}"
                            )
                        return

    async def observe_is_in_air(self, running_tasks):
        """ Monitors whether the drone is flying or not and
        returns after landing """

        was_in_air = False

        async for is_in_air in self.drone.telemetry.in_air():
            if is_in_air:
                was_in_air = is_in_air

            if was_in_air and not is_in_air:
                for task in running_tasks:
                    task.cancel()
                    try:
                        await task
                    except asyncio.CancelledError:
                        pass
                await asyncio.get_event_loop().shutdown_asyncgens()

                return

    async def go_to_search_area(self, lat, lon, alt, yaw):
        self.get_logger().info("-- Going to search area")
        await self.drone.offboard.set_velocity_ned(PositionNedYaw(5.0, 0.0, 0.0, 0.0))

    async def run(self):
        print("Hello World")
        await self.import_and_upload_mission("test.plan")
        await self.flight_check()
        await self.arm()
        await self.start_mission()

        await self.termination_task

def main(args=None):
    rclpy.init(args=args)

    mavsdk_motion = MavsdkMotion()

    rclpy.spin(mavsdk_motion)

    mavsdk_motion.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()