# import matplotlib.pyplot as plt
# import numpy as np
# from pathlib import Path
# import cv2

# from calib_commons.data.load_calib import construct_cameras_intrinsics
# from calib_commons.data.data_pickle import save_to_pickle, load_from_pickle
# from calib_commons.eval_generic_scene import eval_generic_scene
# from calib_commons.scene import SceneType
# from calib_commons.viz import visualization as generic_vizualization
# from calib_commons.utils.detect_board import detect_board_corners, BoardType

# from calib_board.core.checkerboardGeometry import CheckerboardGeometry
# from calib_board.core.checkerboard import CheckerboardMotion
# from calib_board.core.config import ExternalCalibratorConfig
# from calib_board.core.externalCalibrator import ExternalCalibrator, WorldFrame
# from calib_board.core.correspondences import (
#     filter_correspondences_with_track_length,
#     filter_correspondences_with_non_nan_points,
# )
# from calib_board.utils.convert_to_generic import (
#     convert_checker_scene_to_generic_scene,
#     convert_to_generic_correspondences,
# )
# from calib_board.utils.utils import convert_correspondences_array_to_checker_correspondences
# from calib_board.utils import visualization


# # random seed
# np.random.seed(1)

# ############################### USER INTERFACE ####################################

# # PATHS
# images_parent_folder = str(
#     Path(
#         r"/home/fred/dev/orx/orx_middleware/ros2_ws/src/kuka_camera/src/user_applications/calibration_captures/calibration_019"
#     )
# )
# intrinsics_folder = str(
#     Path(
#         r"/home/fred/dev/orx/orx_middleware/ros2_ws/src/kuka_camera/src/user_applications/calibration_captures/calibrate_intrinsics_output/camera_intrinsics"
#     )
# )

# # PRE-PROCESSING PARAMETERS
# # checkerboard_geometry = CheckerboardGeometry(rows = 4, # internal rows
# #                                             columns = 6, # internal columns
# #                                             square_size = 0.165)  # [m]
# checkerboard_geometry = CheckerboardGeometry(
#     rows=8, columns=11, square_size=0.03  # internal rows  # internal columns
# )  # [m]
# board_type = BoardType.CHESSBOARD
# if board_type == BoardType.CHARUCO:
#     charuco_marker_size = 0.123
#     charuco_dictionary = cv2.aruco.DICT_4X4_100
# show_detection_images = True
# save_detection_images = True

# # CALIBRATION PARAMETERS
# external_calibrator_config = ExternalCalibratorConfig(
#     checkerboard_motion=CheckerboardMotion.FREE,  # CheckerboardMotion.PLANAR or CheckerboardMotion.FREE
#     min_track_length=2,  # min number of camera per object (=3D) point
#     checkerboard_geometry=checkerboard_geometry,
#     reprojection_error_threshold=1,  # [pix]
#     min_number_of_valid_observed_points_per_checkerboard_view=10,
#     ba_least_square_ftol=1e-6,  # Non linear Least-Squares
#     least_squares_verbose=2,  # 0: silent, 1: report only final results, 2: report every iteration
#     camera_score_threshold=200,
#     verbose=1,  # 0: only final report, 1: only camera name when added, 2: full verbose
# )
# out_folder_calib = Path("results")
# show_viz = 1
# save_viz = 1
# save_eval_metrics_to_json = 1


# ###################### PRE-PROCESSING: CORNERS DETECTION ###########################

# if board_type == BoardType.CHARUCO:
#     charuco_detector = cv2.aruco.CharucoDetector(
#         cv2.aruco.CharucoBoard(
#             (checkerboard_geometry.columns + 1, checkerboard_geometry.rows + 1),
#             checkerboard_geometry.square_size,
#             charuco_marker_size,
#             cv2.aruco.getPredefinedDictionary(charuco_dictionary),
#         )
#     )
# else:
#     charuco_detector = None

# correspondences_nparray = detect_board_corners(
#     images_parent_folder=images_parent_folder,
#     board_type=board_type,
#     charuco_detector=charuco_detector,
#     columns=checkerboard_geometry.columns,
#     rows=checkerboard_geometry.rows,
#     intrinsics_folder=intrinsics_folder,
#     undistort=True,
#     display=show_detection_images,
#     save_images_with_overlayed_detected_corners=save_detection_images,
# )

# correspondences = convert_correspondences_array_to_checker_correspondences(correspondences_nparray)

# # # save_to_pickle(out_folder_calib / "correspondences_detected.pkl", correspondences)
# # # correspondences = load_from_pickle("results/correspondences_detected.pkl")


# ###################### EXTERNAL CALIBRATION ###########################

# # keep only chessboard views with sufficient corners detected
# correspondences = filter_correspondences_with_non_nan_points(
#     correspondences, external_calibrator_config.min_number_of_valid_observed_points_per_checkerboard_view
# )

# # keep only chessboard with sufficient track length
# correspondences = filter_correspondences_with_track_length(correspondences, external_calibrator_config.min_track_length)

# out_folder_calib.mkdir(parents=True, exist_ok=True)
# intrinsics = construct_cameras_intrinsics(images_parent_folder, intrinsics_folder)

# # Calibrate
# externalCalibrator = ExternalCalibrator(
#     correspondences=correspondences, intrinsics=intrinsics, config=external_calibrator_config
# )
# externalCalibrator.calibrate()
# checkerboard_scene_estimate = externalCalibrator.get_scene(world_frame=WorldFrame.CAM_FIRST_CHOOSEN)
# checkerboard_correspondences = externalCalibrator.correspondences
# checkerboard_scene_estimate.print_cameras_poses()
# generic_scene = convert_checker_scene_to_generic_scene(checkerboard_scene_estimate, scene_type=SceneType.ESTIMATE)
# generic_obsv = convert_to_generic_correspondences(checkerboard_correspondences)

# # Save files
# generic_scene.save_cameras_poses_to_json(out_folder_calib / "camera_poses.json")
# print("camera poses saved to", out_folder_calib / "camera_poses.json")
# scene_estimate_file = out_folder_calib / "scene_estimate.pkl"
# save_to_pickle(scene_estimate_file, generic_scene)
# print("scene estimate saved to", scene_estimate_file)
# correspondences_file = out_folder_calib / "correspondences.pkl"
# save_to_pickle(correspondences_file, generic_obsv)
# print("correspondences saved to", correspondences_file)
# metrics = eval_generic_scene(
#     generic_scene,
#     generic_obsv,
#     camera_groups=None,
#     save_to_json=save_eval_metrics_to_json,
#     output_path=out_folder_calib / "metrics.json",
#     print_=True,
# )
# print("")

# # Visualization
# if show_viz or save_viz:
#     dpi = 300
#     save_path = out_folder_calib / "scene.png"
#     visualization.visualize_scenes(
#         [checkerboard_scene_estimate], show_ids=False, show_fig=show_viz, save_fig=save_viz, save_path=save_path
#     )
#     if save_viz:
#         print("scene visualization saved to", save_path)
#     save_path = out_folder_calib / "2d.png"
#     visualization.visualize_2d(
#         checkerboard_scene_estimate,
#         checkerboard_correspondences,
#         which="both",
#         subplots=True,
#         show_ids=False,
#         show_fig=show_viz,
#         save_fig=save_viz,
#         save_path=save_path,
#     )
#     if save_viz:
#         print("2d visualization saved to", save_path)
#     save_path = out_folder_calib / "2d_errors.png"
#     generic_vizualization.plot_reprojection_errors(
#         scene_estimate=generic_scene,
#         observations=generic_obsv,
#         show_fig=show_viz,
#         save_fig=save_viz,
#         save_path=save_path,
#     )
#     if save_viz:
#         print("2d errors visualization saved to", save_path)

#     if show_viz:
#         def on_key(event):
#             if event.key == 'q': # Quit the program completely
#                 plt.close('all')
#                 exit() # Exit the program
#             elif event.key == 'c': # Close the current window
#                 plt.close()

#         fig = plt.gcf() # Get the current figure
#         fig.canvas.mpl_connect('key_press_event', on_key)
#         plt.show() 

import os
import shutil
import argparse
import numpy as np
from pathlib import Path
import cv2

from calib_commons.data.load_calib import construct_cameras_intrinsics
from calib_commons.data.data_pickle import save_to_pickle, load_from_pickle
from calib_commons.eval_generic_scene import eval_generic_scene
from calib_commons.scene import SceneType
from calib_commons.viz import visualization as generic_vizualization
from calib_commons.utils.detect_board import detect_board_corners, BoardType

from calib_board.core.checkerboardGeometry import CheckerboardGeometry
from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.config import ExternalCalibratorConfig
from calib_board.core.externalCalibrator import ExternalCalibrator, WorldFrame
from calib_board.core.correspondences import (
    filter_correspondences_with_track_length,
    filter_correspondences_with_non_nan_points,
)
from calib_board.utils.convert_to_generic import (
    convert_checker_scene_to_generic_scene,
    convert_to_generic_correspondences,
)
from calib_board.utils.utils import convert_correspondences_array_to_checker_correspondences
from calib_board.utils import visualization


# random seed
np.random.seed(1)

def backup_existing_files(output_folder):
    # Check if there are any files (ignoring subfolders)
    existing_files = [f for f in os.listdir(output_folder) if os.path.isfile(os.path.join(output_folder, f))]
    if existing_files:
        # Check for backup folders
        backup_folders = [folder for folder in os.listdir(output_folder) if folder.startswith("backup_") and os.path.isdir(os.path.join(output_folder, folder))]
        backup_folders.sort()
        if backup_folders:
            last_backup = int(backup_folders[-1].split("_")[1])
            new_backup_folder = os.path.join(output_folder, f"backup_{last_backup + 1:03d}")
        else:
            new_backup_folder = os.path.join(output_folder, "backup_000")
        
        os.makedirs(new_backup_folder, exist_ok=True)

        # Move existing files to the new backup folder
        for file in existing_files:
            os.rename(os.path.join(output_folder, file), os.path.join(new_backup_folder, file))

def main():
    # Argument parsing
    parser = argparse.ArgumentParser(description="External camera calibration using images and intrinsics data.")
    parser.add_argument("--images_folder", type=str, required=True, help="Path to the images folder.")
    parser.add_argument("--intrinsics_folder", type=str, required=True, help="Path to the intrinsics folder.")
    parser.add_argument("--output_folder", type=str, required=True, help="Path to the output folder where results will be saved.")

    # Parsing the arguments
    args = parser.parse_args()

    # Handle backup for existing files in the output folder
    backup_existing_files(args.output_folder)

    # Define intrinsics folder (files only, no subfolders)
    intrinsics_files = [
        os.path.join(args.intrinsics_folder, f) for f in os.listdir(args.intrinsics_folder)
        if os.path.isfile(os.path.join(args.intrinsics_folder, f))
    ]

    # Set up the checkerboard geometry (pre-processed parameters)
    checkerboard_geometry = CheckerboardGeometry(rows=8, columns=11, square_size=0.03)
    board_type = BoardType.CHESSBOARD
    charuco_detector = None  # Since you're using chessboard in the pre-processing, not charuco

    # Detect board corners from images in images_folder
    print(f"Detecting corners from images in {args.images_folder}...")
    correspondences_nparray = detect_board_corners(
        images_parent_folder=args.images_folder,
        board_type=board_type,
        charuco_detector=charuco_detector,
        columns=checkerboard_geometry.columns,
        rows=checkerboard_geometry.rows,
        intrinsics_folder=args.intrinsics_folder,
        undistort=True,
        display=False,  # Set to True if you want to see the detected corners
        save_images_with_overlayed_detected_corners=False,  # Set to True if you want to save images with detected corners
    )

    correspondences = convert_correspondences_array_to_checker_correspondences(correspondences_nparray)

    # External calibration setup
    external_calibrator_config = ExternalCalibratorConfig(
        checkerboard_motion=CheckerboardMotion.FREE,
        min_track_length=2,
        checkerboard_geometry=checkerboard_geometry,
        reprojection_error_threshold=1,
        min_number_of_valid_observed_points_per_checkerboard_view=10,
        ba_least_square_ftol=1e-6,
        least_squares_verbose=2,
        camera_score_threshold=200,
        verbose=1,
    )

    # Load the intrinsics
    print("Loading camera intrinsics...")
    intrinsics = construct_cameras_intrinsics(args.images_folder, args.intrinsics_folder)

    # Run calibration
    print("Starting external calibration...")
    externalCalibrator = ExternalCalibrator(
        correspondences=correspondences, intrinsics=intrinsics, config=external_calibrator_config
    )
    externalCalibrator.calibrate()
    checkerboard_scene_estimate = externalCalibrator.get_scene(world_frame=WorldFrame.CAM_FIRST_CHOOSEN)

    # Save extrinsics data
    out_folder_calib = Path(args.output_folder)
    out_folder_calib.mkdir(parents=True, exist_ok=True)

    generic_scene = convert_checker_scene_to_generic_scene(checkerboard_scene_estimate, scene_type=SceneType.ESTIMATE)
    generic_obsv = convert_to_generic_correspondences(externalCalibrator.correspondences)

    # Save files
    generic_scene.save_cameras_poses_to_json(out_folder_calib / "camera_poses.json")
    save_to_pickle(out_folder_calib / "scene_estimate.pkl", generic_scene)
    save_to_pickle(out_folder_calib / "correspondences.pkl", generic_obsv)

    print(f"Results saved in {out_folder_calib}")

    # Visualization (optional)
    save_path = out_folder_calib / "scene.png"
    visualization.visualize_scenes(
        [checkerboard_scene_estimate], show_ids=False, show_fig=False, save_fig=True, save_path=save_path
    )

    save_path = out_folder_calib / "2d.png"
    visualization.visualize_2d(
        checkerboard_scene_estimate,
        externalCalibrator.correspondences,
        which="both",
        subplots=True,
        show_ids=False,
        show_fig=False,
        save_fig=True,
        save_path=save_path,
    )

    save_path = out_folder_calib / "2d_errors.png"
    generic_vizualization.plot_reprojection_errors(
        scene_estimate=generic_scene,
        observations=generic_obsv,
        show_fig=False,
        save_fig=True,
        save_path=save_path,
    )

    print("Visualizations saved.")

if __name__ == "__main__":
    main()
