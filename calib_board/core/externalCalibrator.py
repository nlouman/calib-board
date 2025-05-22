from typing import Dict, List, Tuple

import numpy as np
import itertools
import cv2
from scipy.optimize import least_squares
import time 
import cProfile
import pstats
import copy
import matplotlib.pyplot as plt

from calib_commons.types import idtype
from calib_commons.utils import se3
from calib_commons.camera import Camera
from calib_commons.intrinsics import Intrinsics
from calib_commons.types import idtype
from calib_commons.utils.se3 import SE3
from calib_commons.world_frame import WorldFrame

from calib_board.core import ba 
from calib_board.core.correspondences import Correspondences, get_conform_views_of_cam, get_tracks, filter_with_track_length
from calib_board.core.checkerboard import CheckerboardMotion
from calib_board.core.observationCheckerboard import ObservationCheckerboard
from calib_board.core.scene import SceneCheckerboard, SceneType
from calib_board.core.checkerboard import Checkerboard
from calib_board.core.config import ExternalCalibratorConfig


class ExternalCalibrator: 

    def __init__(self, 
                 correspondences: Correspondences, 
                 intrinsics: Dict[idtype, Intrinsics],
                 config: ExternalCalibratorConfig,
                 ): 
        
        # data
        self.correspondences = copy.deepcopy(correspondences) # 2d
        self.intrinsics = intrinsics

        # solving parameters
        self.config = config

        # current estimate
        self.estimate = SceneCheckerboard(scene_type=SceneType.ESTIMATE) 

        self.checkers_removed = []

        self.remaining_cameras = set(self.correspondences.keys())
        
    def get_camera_score(self, 
                         cam_id: idtype) -> float:
        
        common_checkers_id = set(self.get_conform_views_of_cam(cam_id).keys()) & self.estimate.get_checkers_ids()
        valid_common_checkers_id = self.filter_with_track_length(common_checkers_id)

        image_points = []
        for checker_id in valid_common_checkers_id:
            image_points.append(self.correspondences[cam_id][checker_id].get_2d_conform())

        if len(image_points):
                image_points = np.vstack(image_points)
                print(f"cam {cam_id} has {len(image_points)} conform points")
                print(f"cam intrinsics {self.intrinsics[cam_id]}")
                score = self.view_score(image_points, self.intrinsics[cam_id].resolution)
        else: 
            score = 0

        return score

    def get_cameras_scores(self): 
        scores = {}
        for cam_id in self.estimate.cameras.keys(): 
            scores[cam_id] = self.get_camera_score(cam_id)
        return scores
    
    def calibrate(self): 

        # initialization 
        initial_camera_id = self.select_initial_camera()
        self.estimate.add_camera(Camera(initial_camera_id, SE3.idendity(), self.intrinsics[initial_camera_id]))
        if self.config.verbose >= 1:
            print("*********** Cam " + str(initial_camera_id) + " added ***********")
        self.add_checkers(initial_camera_id)
        self.remaining_cameras.remove(initial_camera_id)

        # add cameras incrementaly
        while len(self.remaining_cameras): 
            cam_id = self.select_next_best_cam()
            self.add_camera(cam_id)
            self.add_checkers(cam_id)
            self.iterative_filtering()

        print(" ")
        print("##################### CALIBRATION TERMINATED #####################")

        print(f"Chessboards removed: ", self.checkers_removed)
        # compute score of each camera ? -> taking into account only comform views 
        scores = self.get_cameras_scores()
        print("Camera observations scores: ", scores)
        if min(scores.values()) > self.config.camera_score_threshold: 
            print ("Calibration Status: SUCCESS: each camera has sufficient distribution score.")
            return True, scores
        else: 
            print (f"Calibration Status: FAILURE: some cameras do not have a sufficient distribution score.")
            return False, scores

    def get_scene(self, world_frame: WorldFrame) -> SceneCheckerboard:
        if world_frame == WorldFrame.CAM_FIRST_CHOOSEN: 
            T = np.eye(4)
        elif self.estimate.cameras.get(world_frame): 
            T = se3.inv_T(self.estimate.cameras[world_frame].pose.mat)
        # elif world_frame == WorldFrame.CAM_ID_1: 
        #     T = se3.inv_T(self.estimate.scene.cameras[1].pose.mat)
        else: 
            raise ValueError("World Frame not valid.")
        
        cameras = {id: Camera(id, SE3(T @ camera.pose.mat), camera.intrinsics) for id, camera in self.estimate.cameras.items()}
        checkers = {id: Checkerboard(id, SE3(T @ checker.pose.mat), self.config.checkerboard_geometry) for id, checker in self.estimate.checkers.items()}
        return SceneCheckerboard(cameras=cameras, 
                      checkers=checkers, 
                      scene_type=SceneType.ESTIMATE)

    def pnp(self, 
            _2d: np.ndarray, 
            _3d: np.ndarray, 
            K: np.ndarray) -> np.ndarray: 
        """
       Solve PnP using RANSAC, followed by non-linear refinement (minimizing reprojection error) using LM. 

        Args : 
            _2d: np.ndarray, dim: (N, 3)
                image-points
            _3d: np.ndarray, dim: (M, 3)
                 object-points
            K: np.ndarray, dim: (3,3)
                intrinsics matrix
        Returns : 
            T_W_C: np.ndarray, dim: (4, 4)
                pose of the camera in the world frame {w} (the frame of the object-points)
        """
        _, rvec, tvec = cv2.solvePnP(objectPoints=_3d, 
                                     imagePoints=_2d, 
                                     cameraMatrix=K, 
                                     distCoeffs=None)

        T_C_W = se3.T_from_rvec_tvec(rvec, tvec)
        T_W_C = se3.inv_T(T_C_W)
        return T_W_C

    def select_initial_camera(self) -> int:

        cameras_id = self.correspondences.keys()
        arrangements = list(itertools.permutations(cameras_id, 2))

        best_score = -np.inf
        best_cam_id = None

        for arrangement in arrangements:
            cam0_id, cam1_id = arrangement            

            common_checkers_id = set(self.get_conform_views_of_cam(cam0_id).keys()) & set(self.get_conform_views_of_cam(cam1_id).keys())
            valid_common_checkers_id = self.filter_with_track_length(common_checkers_id)

            image_points = []
            for checker_id in valid_common_checkers_id:
                image_points.append(self.correspondences[cam1_id][checker_id]._2d)

            if len(image_points):
                image_points = np.vstack(image_points)
                print(f"cam {cam1_id} has {len(image_points)} conform points")
                print(f"cam intrinsics {self.intrinsics[cam1_id]}")
                score = self.view_score(image_points, self.intrinsics[cam1_id].resolution)
            else: 
                score = 0

            print(f"cam {cam1_id} score: {score}")
            print(f"cam {cam0_id}")

            score = self.view_score(image_points, self.intrinsics[cam1_id].resolution)
            # print("(" + str(cam0Id) + ", " + str(cam1Id) + "): " + str(score))
            if score > best_score:
                best_score = score
                best_cam_id = cam0_id

        return best_cam_id
    
    def select_next_best_cam(self) -> int:
        candidates = self.remaining_cameras

        best_score = -np.inf
        best_cam_id = None

        for cam_id in candidates:
            score = self.get_camera_score(cam_id)
            
            # print("(" + str(cam0Id) + ", " + str(cam1Id) + "): " + str(score))
            if score > best_score:
                best_score = score
                best_cam_id = cam_id

        return best_cam_id
    
    def view_score(self, 
                   image_points: np.ndarray, 
                   image_resolution: Tuple):
        
        print( f"image points: {image_points.shape}")
        print( f"image resolution: {image_resolution}")
        s = 0
        L = 3
        width, height = image_resolution
        for l in range(1, L+1):
            K_l = 2**l
            w_l = K_l  # Assuming w_l = K_l is correct
            grid = np.zeros((K_l, K_l), dtype=bool)
            for point in image_points:
                if np.isnan(point).any():
                    continue
                x = int(np.ceil(K_l * point[0] / width)) - 1
                y = int(np.ceil(K_l * point[1] / height)) - 1
                if grid[x, y] == False:
                    grid[x, y] = True  # Mark the cell as full
                    s += w_l  # Increase the score
        return s
    
    def add_camera(self, cam_id: idtype) -> None: 
        self.remaining_cameras.remove(cam_id)

        K = self.intrinsics[cam_id].K   


        common_checkers_id = set(self.get_conform_views_of_cam(cam_id).keys()) & self.estimate.get_checkers_ids()
        valid_common_checkers_id = self.filter_with_track_length(common_checkers_id)

        if not valid_common_checkers_id:
            return 
        
       

        best_num_inliers = 0
        best_inliers = None
        best_cam_pose = None

        for checker_id in valid_common_checkers_id:
            _2d = self.correspondences[cam_id][checker_id]._2d
            # _3d = 

            valid_mask = ~np.isnan(_2d).any(axis=1)
            _2d = _2d[valid_mask]
            _3d = self.estimate.checkers[checker_id].get_3d_points_in_world()[valid_mask]

            T_W_C = self.pnp(_2d, _3d, K)
            
            inliers = []
            # print("camera pose estimated using checker " + str(checkerId))
            # print(T_W_C)

            for checker_id_to_classify in valid_common_checkers_id: 
                _3d = self.estimate.checkers[checker_id_to_classify].get_3d_points_in_world()
                _2d_reprojection = self.intrinsics[cam_id].reproject(T_W_C, _3d)
                _2d = self.correspondences[cam_id][checker_id_to_classify]._2d
                errors = np.sqrt(np.sum((_2d - _2d_reprojection)**2, axis=1))
                # mean_error = np.mean(errors)
                # print(f"error of checker {checkerIdToClassify}: {meanError}")
                
                # is_inlier = mean_error < self.config.checkerboard_reprojection_error_threshold
                errors = errors[~np.isnan(errors)]
                conformity_mask = errors < self.config.reprojection_error_threshold
                # outlier_ratio_intra_checkerboard = np.sum(~conformity_mask) / len(conformity_mask)
                # is_inlier = outlier_ratio_intra_checkerboard < self.config.max_outlier_ratio_intra_checkerboard
                is_inlier = np.sum(conformity_mask) >= self.config.min_number_of_valid_observed_points_per_checkerboard_view
                inliers.append(is_inlier)

            num_inliers = np.sum(inliers)
            # print("num of inliers: " + str(numInliers))
            if num_inliers > best_num_inliers: 
                best_num_inliers = num_inliers
                best_inliers = inliers
                best_cam_pose = T_W_C
                best_checker_id = checker_id
            
        # non linear refinement based on biggest set of inliers 
        _2d = []
        _3d = []
        for i, checker_id in enumerate(list(valid_common_checkers_id)): 
            if best_inliers[i]:
                _2d.append(self.correspondences[cam_id][checker_id]._2d)
                _3d.append(self.estimate.checkers[checker_id].get_3d_points_in_world())
        _2d = np.concatenate(_2d, axis=0)
        _3d = np.concatenate(_3d, axis=0)

        # print(f"best pose is the one computed from checker {bestCheckerId}")
        # print(bestCamPose)
        # print(f"best inliers: {np.where(np.array(bestInliers))[0]}")
        T_C_W0 = se3.inv_T(best_cam_pose)
        rvec0, tvec0 = se3.rvec_tvec_from_T(T_C_W0)
        ##
        rvec_, tvec_ = cv2.solvePnPRefineLM(objectPoints=_3d, 
                                            imagePoints=_2d, 
                                            cameraMatrix=self.intrinsics[cam_id].K, 
                                            distCoeffs=None, 
                                            rvec=rvec0[:,np.newaxis], 
                                            tvec=tvec0[:,np.newaxis], )
        T_C_W = se3.T_from_rvec_tvec(rvec_, tvec_)
        T_W_C = se3.inv_T(T_C_W)

        self.estimate.add_camera(Camera(cam_id, SE3(T_W_C), self.intrinsics[cam_id]))
        # if self.config.display: 
        if self.config.verbose >= 1:
            if self.config.verbose >= 2:
                print(" ")
            print("*********** Cam " + str(cam_id) + " added ***********")
            if self.config.verbose >= 2:
                print(f"had {len(valid_common_checkers_id)} views of checkers in common")
                print(f"initial pose refined on {best_num_inliers} checkers")
           
    def add_checkers(self, camera_id: idtype) -> None: 
        new_checker_ids = set(self.get_conform_views_of_cam(camera_id).keys()) - self.estimate.get_checkers_ids()
        valid_checker_ids = self.filter_with_track_length(new_checker_ids)
        for checker_id in valid_checker_ids:
            _2d = self.correspondences[camera_id][checker_id]._2d
            valid_mask = ~np.isnan(_2d).any(axis=1)
            _2d = _2d[valid_mask]
            _3d = self.config.checkerboard_geometry._3d[valid_mask]
            if np.sum(valid_mask) < 6: 
                print()
            # _3d = self.config.checkerboard_geometry._3d
            T_B_C = self.pnp(_2d, _3d, self.intrinsics[camera_id].K)
            T_W_C = self.estimate.cameras[camera_id].pose.mat
            T_W_B = T_W_C @ se3.inv_T(T_B_C)
            self.estimate.add_checker(Checkerboard(checker_id, SE3(T_W_B), self.config.checkerboard_geometry))

        if self.config.verbose >= 2:
            if len(valid_checker_ids):
                print("New checkers: " + str(sorted(valid_checker_ids)))
            else: 
                print("New checkers: None")

    def iterative_filtering(self): 
        if self.config.verbose >= 2:
            print(" ")
            print("----- Iterative Filtering -----")
        iter = 1
        continue_ = True
        while continue_:
            if self.config.verbose >= 2:
                print(" ")
                print(" ")
                print(f"----- Iteration: {iter} -----" )
                print("** BA: **")
            iter += 1

            self.bundle_adjustment()
            if self.config.verbose >= 2:
                print(" ")
                print("** Filtering: **")

            # if self.config.display_reprojection_errors:
            #     self.display_histogram_reprojection_errors()
            continue_ = self.filtering()
           
  

    def filtering(self) -> bool: 
        num_view_filtered = 0
        checker_removed = []
        
        checkers_ids = sorted(list(self.estimate.checkers.keys()))

        for camera in self.estimate.cameras.values(): 
            for checker_id in checkers_ids: 
                checker = self.estimate.checkers.get(checker_id)
                if checker:
                    observation = self.correspondences[camera.id].get(checker.id)
                    if observation and observation._is_conform:
                        _2d_reprojection = camera.reproject(checker.get_3d_points_in_world())
                        errors_xy = observation._2d - _2d_reprojection
                        errors = np.sqrt(np.sum(errors_xy**2, axis=1))
                        self.correspondences[camera.id][checker_id]._conformity_mask = errors < self.config.reprojection_error_threshold
                        num_conform_points = np.sum(errors < self.config.reprojection_error_threshold)
                        is_obsv_conform = num_conform_points >= self.config.min_number_of_valid_observed_points_per_checkerboard_view
                        if not is_obsv_conform: 
                            num_view_filtered += 1
                            self.correspondences[camera.id][checker.id]._is_conform = False 
                            if len(self.get_tracks()[checker.id]) < self.config.min_track_length:
                                checker_removed.append(checker.id)
                                self.checkers_removed.append(checker.id)   
                                del self.estimate.checkers[checker.id]

            
        if self.config.verbose >= 2:
            print(f" -> Number of views filtered: {num_view_filtered}")
            print(f" -> Checkers removed from estimate: {sorted(checker_removed)}")
        return num_view_filtered > 0
    

    def bundle_adjustment(self): 
        x0 = self.parametrization_from_estimate(self.estimate, self.config.checkerboard_motion)

        profile = False
        if profile:
            profiler = cProfile.Profile()
            profiler.enable()
        t0 = time.time()

        num_cameras = self.estimate.get_num_cameras()
        num_checkerboards = self.estimate.get_num_checkers()
        intrinsics_array = np.zeros((3,3,num_cameras))
        for i, camera in enumerate(self.estimate.cameras.values()): 
            intrinsics_array[:,:,i] = self.intrinsics[camera.id].K

        num_cameras = self.estimate.get_num_cameras()
        num_checkers = self.estimate.get_num_checkers()
        num_corners = self.config.checkerboard_geometry.get_num_corners()

        observations = np.full((2, num_cameras, num_checkers, num_corners), np.nan)
        cameras_ids_obs = np.full((2, num_cameras, num_checkers, num_corners), np.nan, dtype=int)
        checker_ids_obs = np.full((2, num_cameras, num_checkers, num_corners), np.nan, dtype=int)

        for i, camera in enumerate(self.estimate.cameras.values()): 
            for k, checker in enumerate(self.estimate.checkers.values()):
                observation = self.correspondences[camera.id].get(checker.id)
                if observation and observation._is_conform:
                    _2d_masked = np.zeros_like(observation._2d.T)
                    _2d_masked[:,:] = observation._2d.T[:,:]
                    _2d_masked[:, ~observation._conformity_mask] = np.nan
                    observations[:, i, k, :] = _2d_masked
                    cameras_ids_obs[:, i, k, :] = i
                    checker_ids_obs[:, i, k, :] = k
                 
        observations = observations.ravel()
        valid_mask = ~np.isnan(observations)
        cameras_ids_flat = cameras_ids_obs.ravel()
        checker_ids_flat = checker_ids_obs.ravel()

        observations = observations[valid_mask]
        cameras_ids = cameras_ids_flat[valid_mask]
        checker_ids = checker_ids_flat[valid_mask]

      
        A = ba.compute_jacobian_sparsity_pattern(len(observations), cameras_ids, checker_ids, num_cameras, num_checkers)
        # plt.spy(A, markersize=1)
        # plt.show()

        results = least_squares(fun = ba.cost_function_ba, 
                                x0 =            x0, 
                                x_scale =       'jac',
                                jac_sparsity =  A,
                                verbose =       self.config.least_squares_verbose, 
                                ftol =          self.config.ba_least_square_ftol,
                                method =        'trf', 
                                args =          (num_cameras, num_checkerboards, self.config.checkerboard_motion, self.config.checkerboard_geometry._3d_augmented, intrinsics_array, observations, valid_mask)
                                )
               
        t1 = time.time()
        if self.config.verbose >= 2:
            print(f"BA optimization time: {t1-t0:>3.2f} [s]")
        if profile:
            profiler.disable()
            stats = pstats.Stats(profiler).sort_stats('cumtime')
            stats.print_stats()
        
        self.estimate = self.update_estimate_from_parametrization(x=results.x, 
                                                num_cameras=num_cameras, 
                                                num_checkers=num_checkerboards)
        
    
         
    def update_estimate_from_parametrization(self, 
                                             x: np.ndarray, 
                                             num_cameras: int, 
                                             num_checkers: int):
       
        camera_poses, checker_poses = ba.extract_all_poses_from_x(x, num_cameras, num_checkers, self.config.checkerboard_motion)
        
        estimate = copy.deepcopy(self.estimate)
        for i, camera_id in enumerate(estimate.cameras.keys()):
            estimate.cameras[camera_id].pose = SE3(camera_poses[:,:,i])

        for k, checker_id in enumerate(estimate.checkers.keys()):
            estimate.checkers[checker_id].pose = SE3(checker_poses[:,:,k])

        return estimate

    def parametrization_from_estimate(self, 
                                      estimate: SceneCheckerboard, 
                                      checkerboard_motion: CheckerboardMotion):
        x = []
        
        for camera in estimate.cameras.values(): 
            # we skip the first choosen camera (it's pose is identity)
            if camera.id == next(iter(estimate.cameras)): # next(iter(dict)) returns the first key of the dict
                # print(f"first cam id: {camera.id}")
                idendity_camera_id = camera.id
                continue
            x.append(se3.q_from_T(camera.pose.mat))
        
        first_added_checker_id = next(iter(estimate.checkers))
        for checker in estimate.checkers.values(): 
            if checker.id == first_added_checker_id: 
                # print(f"first checker id: {checker.id}")

                # pose of the first checker wrt world
                # add
                x.append(se3.q_from_T(checker.pose.mat))

            else: 
                # pose of the checker wrt to the first checker
                # this parametrization depends on the checkerboard motion case
                # if planar -> 3 d.o.f, free -> 6 d.o.f.
                T_B1_B = estimate.checkers[first_added_checker_id].pose.inv().mat @ checker.pose.mat
                q = se3.q_from_T(T_B1_B)
                size = 6
                if checkerboard_motion == CheckerboardMotion.PLANAR: 
                    q = q[[0, 3, 4]] # extract [theta_z, t_x, t_y]
                    size = 3
                # add
                x.append(q)
                
        x = np.concatenate(x, axis=0)
        return x

    # getters

    def get_camera_ids(self) -> set[idtype]: 
        return set(self.correspondences.keys())

    def get_num_cameras(self) -> int: 
        return len(self.correspondences)
    
    def get_remaining_camera_ids(self) -> set[idtype]: 
        return self.get_camera_ids() - self.estimate.get_camera_ids()
    
    def get_conform_views_of_cam(self, cam_id) -> Dict[idtype, ObservationCheckerboard]: 
        return get_conform_views_of_cam(cam_id, self.correspondences)

    def get_tracks(self) -> Dict[idtype, set[idtype]]: 
       return get_tracks(self.correspondences)

    # manipulations

    def filter_with_track_length(self, checker_ids) -> set[idtype]: 
        return filter_with_track_length(self.correspondences, checker_ids, self.config.min_track_length)

    def print_errors(self): 
        for camera in self.estimate.cameras.values(): 
            for checker_id in sorted(list(self.estimate.checkers.keys())): # list to create a copy of the values so that when removing an element we do not raise an error.
                checker = self.estimate.checkers[checker_id]
                observation = self.correspondences[camera.id].get(checker.id)
                if observation and observation._is_conform:
                    _2d_reprojection = camera.reproject(checker.get_3d_points_in_world())
                    errors_xy = observation._2d - _2d_reprojection
                    errors = np.sqrt(np.sum(errors_xy**2, axis=1))
                    mean_error = np.mean(errors)
                    # if self.config.display_reprojection_errors:
                    #     print(f"view of checker {checker.id:>3} in cam {camera.id} error: {mean_error:.2f} [pix]")

    def display_histogram_reprojection_errors(self): 
        errors = self.estimate.reprojection_error_per_view(self.correspondences)
        for cam_id in errors.keys(): 
            error_for_this_cam = errors[cam_id]
            errors_values = list(error_for_this_cam.values())
            sorted_dict = sorted(error_for_this_cam.items(), key=lambda item: item[1])

            sorted_dict = dict(sorted_dict)
            print(f"cam {cam_id}")
            output = ""

            pad = len(str(max(sorted_dict.keys())))
            for key, value in sorted_dict.items():
                output += f"{key:<{pad}}: {value:.2f}, "

            # Remove the trailing comma and space from the last item
            output = output.rstrip(', ')
            print(output)

            plt.figure()
            plt.hist(errors_values, bins=30, color='blue', alpha=0.7)  # 'bins' defines how many sections to divide the data into
            plt.title(f"Error hist for cam {cam_id}")
            plt.xlabel('error')
            plt.ylabel('f')
