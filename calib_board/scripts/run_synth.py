import matplotlib.pyplot as plt
import numpy as np 
from pathlib import Path

from calib_commons.data.data_pickle import save_to_pickle
from calib_commons.eval_generic_scene import eval_generic_scene
from calib_commons.scene import SceneType
from calib_commons.viz import visualization as generic_vizualization

from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.externalCalibrator import ExternalCalibrator, WorldFrame
from calib_board.utils.sceneGenerator import SceneGenerator
from calib_board.core.correspondences import filter_correspondences_with_track_length, filter_correspondences_with_non_nan_points
from calib_board.utils import visualization
from calib_board.utils.convert_to_generic import convert_checker_scene_to_generic_scene, convert_to_generic_correspondences
from calib_board.core.config import ExternalCalibratorConfig

# random seed
np.random.seed(1)


############################### USER INTERFACE ####################################

# SYNTHETIC SCENE GENERATION PARAMETERS
numCheckerboards = 100
checkerboard_motion = CheckerboardMotion.FREE # CheckerboardMotion.PLANAR or CheckerboardMotion.FREE
checkerboard_motion_range = 0.7 # [m]
num_cameras = 6
distance_cameras = 2 # [m]
tilt_cameras = 45 * np.pi / 180 # [rad]
noise_std = 0.5 # [pix]
checkerboard_geometry = CheckerboardGeometry(rows = 4, # internal rows
                                            columns = 6, # internal columns
                                            square_size = 0.165)  # [m]

# CALIBRATION PARAMETERS
external_calibrator_config = ExternalCalibratorConfig(
    checkerboard_motion = CheckerboardMotion.FREE, # CheckerboardMotion.PLANAR or CheckerboardMotion.FREE
    min_track_length = 2, # min number of camera per object (=3D) point
    checkerboard_geometry = checkerboard_geometry,
    reprojection_error_threshold = 1, # [pix]
    min_number_of_valid_observed_points_per_checkerboard_view = 10,
    ba_least_square_ftol = 1e-6, # Non linear Least-Squares 
    least_squares_verbose = 0, # 0: silent, 1: report only final results, 2: report every iteration
    camera_score_threshold = 200,
    verbose = 1 # 0: only final report, 1: only camera name when added, 2: full verbose
)
out_folder_calib = Path("results")
show_viz = 1
save_viz = 0
save_eval_metrics_to_json = 1


###################### SYNTHETIC SCENE GENERATION ###########################
sceneGenerator = SceneGenerator(num_checkerboards=numCheckerboards, 
                                checkerboard_geometry=external_calibrator_config.checkerboard_geometry, 
                                checkerboard_motion=checkerboard_motion,
                                checkerboard_motion_range=checkerboard_motion_range,
                                num_cameras=num_cameras, 
                                distance_cameras=distance_cameras, 
                                tilt_cameras=tilt_cameras, 
                                min_track_length=external_calibrator_config.min_track_length, 
                                noise_std=noise_std                                
                                )
synthetic_scene = sceneGenerator.generateScene()
correspondences = synthetic_scene.get_correspondences()
intrinsics = synthetic_scene.get_intrinsics()


###################### EXTERNAL CALIBRATION ###########################

# keep only chessboard views with sufficient corners detected
correspondences = filter_correspondences_with_non_nan_points(correspondences, external_calibrator_config.min_number_of_valid_observed_points_per_checkerboard_view)
# keep only chessboard with sufficient track length
correspondences = filter_correspondences_with_track_length(correspondences, external_calibrator_config.min_track_length)

out_folder_calib.mkdir(parents=True, exist_ok=True)

# Calibrate
externalCalibrator = ExternalCalibrator(correspondences=correspondences, 
                                        intrinsics=intrinsics, 
                                        config=external_calibrator_config
                                        )
externalCalibrator.calibrate()
checkerboard_scene_estimate = externalCalibrator.get_scene(world_frame=WorldFrame.CAM_FIRST_CHOOSEN)
checkerboard_correspondences = externalCalibrator.correspondences
checkerboard_scene_estimate.print_cameras_poses()
generic_scene = convert_checker_scene_to_generic_scene(checkerboard_scene_estimate, scene_type=SceneType.ESTIMATE)
generic_obsv = convert_to_generic_correspondences(checkerboard_correspondences)

# Save files
generic_scene.save_cameras_poses_to_json(out_folder_calib / "camera_poses.json")
print("camera poses saved to", out_folder_calib / "camera_poses.json")
scene_estimate_file = out_folder_calib / "scene_estimate.pkl"
save_to_pickle(scene_estimate_file, generic_scene)
print("scene estimate saved to", scene_estimate_file)
correspondences_file = out_folder_calib / "correspondences.pkl"
save_to_pickle(correspondences_file, generic_obsv)
print("correspondences saved to", correspondences_file)
metrics = eval_generic_scene(generic_scene, generic_obsv, camera_groups=None, save_to_json=save_eval_metrics_to_json, output_path=out_folder_calib / "metrics.json", print_ = True)
print("")

# Visualization
if show_viz or save_viz:
    dpi = 300
    save_path = out_folder_calib / "scene.png"
    visualization.visualize_scenes([checkerboard_scene_estimate], show_ids=False, show_fig=show_viz, save_fig=save_viz, save_path=save_path)
    if save_viz:
        print("scene visualization saved to", save_path)
    save_path = out_folder_calib / "2d.png"
    visualization.visualize_2d(checkerboard_scene_estimate, checkerboard_correspondences, which="both", subplots=True, show_ids=False, show_fig=show_viz, save_fig=save_viz, save_path=save_path)
    if save_viz:
        print("2d visualization saved to", save_path)
    save_path = out_folder_calib / "2d_errors.png"
    generic_vizualization.plot_reprojection_errors(scene_estimate=generic_scene, 
                                        observations=generic_obsv, 
                                        show_fig=show_viz,
                                        save_fig=save_viz, 
                                        save_path=save_path)
    if save_viz:
        print("2d errors visualization saved to", save_path)

    if show_viz:
        plt.show()


  