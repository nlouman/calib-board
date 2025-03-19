from typing import Dict, List
import numpy as np

from calib_board.core.observationCheckerboard import ObservationCheckerboard
from calib_commons.types import idtype


Correspondences = Dict[idtype, Dict[idtype, ObservationCheckerboard]]

def get_conform_views_of_cam(cam_id: idtype, 
                             correspondences: Correspondences) -> Dict[idtype, ObservationCheckerboard]: 
    return {key: obs for key, obs in correspondences[cam_id].items() if obs._is_conform}

def get_tracks(correspondences) -> Dict[idtype, set[idtype]]: 
    tracks = {}
    for camera_id, observations in correspondences.items():
        for checker_id, observation in observations.items():
            if observation._is_conform:
                if checker_id in tracks:
                    tracks[checker_id].add(camera_id)
                else:
                    tracks[checker_id] = {camera_id}
            else: 
                if checker_id in tracks:
                    pass
                else:
                    tracks[checker_id] = set()

    return tracks

def filter_with_track_length(correspondences, checker_ids, min_track_length) -> set[idtype]: 
    tracks = get_tracks(correspondences)
    checkers_with_long_enough_track = set([checker_id for checker_id, track in tracks.items() if len(track) >= min_track_length])
    valid_checker_ids = checker_ids & checkers_with_long_enough_track
    return valid_checker_ids

def filter_correspondences_with_track_length(correspondences: Correspondences, 
                                             min_track_length: int) -> Correspondences: 
    tracks = get_tracks(correspondences)
    checkers_with_long_enough_track = set([checker_id for checker_id, track in tracks.items() if len(track) >= min_track_length])

    correspondences_filtered = {}
    for camera_id, observations in correspondences.items():
        correspondences_filtered[camera_id] = {}
        for checker_id, observation in observations.items():
            if checker_id in checkers_with_long_enough_track:
                correspondences_filtered[camera_id][checker_id] = observation
    return correspondences_filtered

# Filter correspondences to keep only those with a sufficient number of non-NaN 2D points
def filter_correspondences_with_non_nan_points(correspondences, threshold):
    filtered_correspondences = {}
    for cam_name, cam_correspondences in correspondences.items():
        filtered_cam_correspondences = {}
        for id in cam_correspondences:
            checkerboard_obs = cam_correspondences[id]
            non_nan_lines = np.sum(~np.isnan(checkerboard_obs._2d).any(axis=1))
            if non_nan_lines >= threshold:
                filtered_cam_correspondences[id] = checkerboard_obs
            else: 
                print("discard id", id, "from camera", cam_name, "because it has only", non_nan_lines, "non-NaN 2D points")
        filtered_correspondences[cam_name] = filtered_cam_correspondences
    return filtered_correspondences