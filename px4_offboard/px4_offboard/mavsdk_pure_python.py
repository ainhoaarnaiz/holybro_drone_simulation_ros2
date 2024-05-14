import asyncio
 
from mavsdk import System
import mavsdk.mission_raw
from mavsdk.offboard import PositionNedYaw, OffboardError
 
async def run():
    drone = System()
    await drone.connect()
 
    print("Ẁaiting for drone to connect...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"-- Connected to drone!")
            break
 
    print_mission_progress_task = asyncio.ensure_future(
        print_mission_progress(drone))
 
    running_tasks = [print_mission_progress_task]
    termination_task = asyncio.ensure_future(
        observe_is_in_air(drone, running_tasks))
 
    mission_import_data = await drone.mission_raw.import_qgroundcontrol_mission("task2.plan")
    print(f"{len(mission_import_data.mission_items)} mission items imported")
    await drone.mission_raw.upload_mission(mission_import_data.mission_items)
    print("Mission uploaded")
 
    print("Waiting for drone to have a global position estimate...")
    async for health in drone.telemetry.health():
        if health.is_global_position_ok and health.is_home_position_ok:
            print("-- Global position estimate OK")
            break
 
    print("-- Arming")
    await drone.action.arm()
 
    print("-- Starting mission")
    await drone.mission_raw.start_mission()
 
    
 
    await termination_task
 
async def print_mission_progress(drone):
    async for mission_progress in drone.mission_raw.mission_progress():
        print(f"Mission progress: "
              f"{mission_progress.current}/"
              f"{mission_progress.total}")
        
        # Check if the mission is completed
        if mission_progress.current == mission_progress.total:
            async for flight_mode in drone.telemetry.flight_mode():
                if str(flight_mode) == "HOLD":
                    # Start offboard mode
                    print("Drone is in hold mode and mission is completed.")
 
                    await drone.offboard.set_position_ned(PositionNedYaw(0.0, 0.0, 0.0, 0.0))
 
                    print("-- Starting offboard")
                    try:
                        await drone.offboard.start()
 
                        # Hold in offboard mode for 5 seconds
                        await asyncio.sleep(5)
                        
                    except OffboardError as error:
                        print(f"Starting offboard mode failed \
                                with error code: {error._result.result}")
                    return
 
async def observe_is_in_air(drone, running_tasks):
    """ Monitors whether the drone is flying or not and
    returns after landing """
 
    was_in_air = False
 
    async for is_in_air in drone.telemetry.in_air():
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
 
if __name__ == "__main__":
    asyncio.run(run())