from typing import List
import os
import matplotlib.pyplot as plt
from matplotlib import rcParams
import math 
import numpy as np

from calib_board.core.scene import SceneCheckerboard, SceneType
from calib_board.core.correspondences import Correspondences
from calib_commons.types import idtype

# from boardCal.utils.evaluate_old import reprojection_error_per_view
from calib_commons.viz.visualization_tools import get_color_from_id, get_color_from_error

# from boardCal.utils.evaluate_old import get_errors_x_y_in_cam


MODIFIERS = {SceneType.SYNTHETIC: r"\overline", 
             SceneType.ESTIMATE: r"\hat"}

MARKERS_OBSERVATIONS = {SceneType.SYNTHETIC: '+', 
                        SceneType.ESTIMATE: 'x'}

def get_coords(scene: SceneCheckerboard, x,y,z): 
    # x_ = []
    # y_ = []
    # z_ = []

    for camera in scene.cameras.values(): 
        x.append(camera.pose.get_x())
        y.append(camera.pose.get_y())
        z.append(camera.pose.get_z())

    for checker in scene.checkers.values(): 
        corners = checker.get_3d_points_in_world()
        for i in range(corners.shape[0]): 
            x.append(corners[i,0])
            y.append(corners[i,1])
            z.append(corners[i,2])

    # x_min = min(x_)
    # x_max = max(x_)

    # y_min = min(y_)
    # y_max = max(y_)

    # z_min = min(z_)
    # z_max = max(z_)

    # print("x: ", x_min, x_max)
    # print("y: ", y_min, y_max)
    # print("z: ", z_min, z_max)

    # min_coords = min([x_min, y_min, z_min])
    # max_coords = max([x_max, y_max, z_max])
    # print("overall:", min_coords, max_coords)

    

    # return min_coords, max_coords

 
def init_ax(ax): 
    ax.set_aspect("auto")
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')

def visualize_scenes(scenes: List[SceneCheckerboard], 
                     show_ids = False,
                     show_fig = True, 
                save_fig = False, 
                save_path = None,
                elev = None, 
                azim = None) -> None: 
    fig = plt.figure(figsize=(6, 6)) 
    rcParams['text.usetex'] = True
    ax = fig.add_subplot(projection='3d')
    if elev and azim:
        ax.view_init(elev=elev, azim=azim)  # Change elev and azim as needed
    # xmin = np.inf
    # ymin = np.inf
    # zmin = np.inf

    # xmax = -np.inf
    x = []
    y = []
    z = []

    for scene in scenes: 
        get_coords(scene, x,y,z)

    x_min = min(x)                          
    x_max = max(x)

    y_min = min(y)
    y_max = max(y)

    z_min = min(z)
    z_max = max(z)

    
        
    min_coords = min([x_min, y_min, z_min])
    max_coords = max([x_max, y_max, z_max])

    # print("x: ", x_min, x_max)
    # print("y: ", y_min, y_max)
    # print("z: ", z_min, z_max)
    size_x = x_max-x_min
    size_y = y_max-y_min
    size_z = z_max-z_min

    size = max([size_x, size_y, size_z])
    # print("size ", size)
    x_mid = (x_max+x_min)/2
    y_mid = (y_max+y_min)/2
    z_mid = (z_max+z_min)/2

    xlim = [x_mid-size/2, x_mid+size/2]
    ylim = [y_mid-size/2, y_mid+size/2]
    zlim = [z_mid-size/2, z_mid+size/2]

    ax.set_xlim(xlim)
    ax.set_ylim(ylim)
    ax.set_zlim(zlim)
    ax.set_xlabel('x')
    ax.set_ylabel('y')
    ax.set_zlabel('z')   
    ax.set_box_aspect([1, 1, 1]) 
    ax.set_title('3D Scenes', fontsize=14, pad=50)   

    for scene in scenes: 
        plot_scene(scene, show_ids)
        if save_fig:
                # Create the directory if it does not exist
                os.makedirs(os.path.dirname(save_path), exist_ok=True)
                fig.savefig(save_path, dpi=300, bbox_inches='tight')

    if show_fig:
        plt.show(block=False)

    # plt.show(block=False)

def visualize_2d(scene: SceneCheckerboard,
                 observations: Correspondences = None, 
                 which = "both",
                 subplots=True, 
                show_fig = True, 
                save_fig = False, 
                save_path = None, 
                show_ids=False) -> None: 
    
    if scene:
        cameras_id = scene.cameras.keys() 
    elif observations: 
        cameras_id = observations.keys()
    else: 
        raise ValueError("scenes or observations must be provided.")
    
    
    if not subplots: # new figure for each camera
        for camera_id in cameras_id:
            fig = plt.figure()
            res_x = scene.cameras[camera_id].intrinsics.resolution[0]
            res_y = scene.cameras[camera_id].intrinsics.resolution[1]
            plt.xlim(0, res_x)
            plt.ylim(res_y, 0)
        
            ax = plt.gca()
            aspect_ratio = res_y / res_x
            ax.set_aspect(aspect_ratio / ax.get_data_ratio())

            escaped_cam_id = latex_escape_underscores(str(camera_id))
            title = rf"Observations in cam ${escaped_cam_id}$"
            plt.title(title)

            plt.show(block=False)
    else: # subplots
        n = len(cameras_id)
        cols = math.ceil(math.sqrt(n))
        rows = math.ceil(n / cols)
        fig_width = cols * 4  # Each subplot has a width of 4 inches (adjust as needed)
        fig_height = rows * 3  # Each subplot has a height of 3 inches (adjust as needed)

        # Create a figure with dynamic size
        fig, axs = plt.subplots(rows, cols, figsize=(fig_width, fig_height))       
        axs = np.array(axs).reshape(-1) if n > 1 else [axs]
        axs_used = [False for ax in axs]

        ax_index = 0
        for camera_id in cameras_id:
            ax = axs[ax_index]
            axs_used[ax_index] = True
            ax_index += 1
           
            res_x = scene.cameras[camera_id].intrinsics.resolution[0]
            res_y = scene.cameras[camera_id].intrinsics.resolution[1]
            ax.set_xlim((0, res_x))
            ax.set_ylim((res_y, 0))
            aspect_ratio = res_y / res_x
            ax.set_aspect(aspect_ratio / ax.get_data_ratio())

            escaped_cam_id = latex_escape_underscores(str(camera_id))
            title = rf"Observations in cam ${escaped_cam_id}$"
            ax.set_title(title)
    
            
            checkers_id = observations[camera_id].keys()
            _2d_observations_conform = {}
            _2d_observations_nonconform = {}
            _2d_observations_nonconform_by_discard_of_chessboard = {}

            _2d_reprojections_conform = {}
            _2d_reprojections_nonconform = {}
            _2d_reprojections_nonconform_by_discard_of_chessboard = {}


            for checker_id in checkers_id:
                if observations[camera_id][checker_id]._is_conform:
                    mask_not_nan = ~np.isnan(observations[camera_id][checker_id]._2d).any(axis=1)

                    conformity_mask = observations[camera_id][checker_id]._conformity_mask

                    tmp = observations[camera_id][checker_id]._2d[conformity_mask & mask_not_nan,:]
                    if np.isnan(tmp).any():
                        print(f"NaN values found in observations for checker_id {checker_id} in camera_id {camera_id}")
                    _2d_observations_conform[checker_id] = tmp 

                    _2d_observations_nonconform[checker_id] = observations[camera_id][checker_id]._2d[~conformity_mask & mask_not_nan, :]
                    # _2d_observations_nonconform[checker_id] = observations[camera_id][checker_id]._2d[~conformity_mask,:]

                    if scene.reprojections[camera_id].get(checker_id):
                        tmp = scene.reprojections[camera_id][checker_id]._2d[conformity_mask& mask_not_nan,:]
                        if np.isnan(tmp).any():
                            print(f"NaN values found in reproj for checker_id {checker_id} in camera_id {camera_id}") 
                        _2d_reprojections_conform[checker_id] = tmp
                        _2d_reprojections_nonconform[checker_id] = scene.reprojections[camera_id][checker_id]._2d[~conformity_mask & mask_not_nan,:]

                    # _2d_observations_conform.append(_2d_observation_conform)
                    # _2d_observations_nonconform.append(_2d_observation_nonconform)
                    # _2d_reprojections_conform.append(_2d_reprojection_conform)
                    # _2d_reprojections_nonconform.append(_2d_reprojection_nonconform)
                else:
                    conformity_mask = observations[camera_id][checker_id]._conformity_mask
                    mask_not_nan = ~np.isnan(observations[camera_id][checker_id]._2d).any(axis=1)
                    _2d_observations_nonconform[checker_id] = observations[camera_id][checker_id]._2d[~conformity_mask & mask_not_nan,:]
                    _2d_observations_nonconform_by_discard_of_chessboard[checker_id] = observations[camera_id][checker_id]._2d[conformity_mask& mask_not_nan,:]

                    if scene.reprojections[camera_id].get(checker_id):
                        _2d_reprojections_nonconform[checker_id] = scene.reprojections[camera_id][checker_id]._2d[~conformity_mask & mask_not_nan,:]
                        _2d_reprojections_nonconform_by_discard_of_chessboard[checker_id] = scene.reprojections[camera_id][checker_id]._2d[conformity_mask& mask_not_nan,:]          


            if show_ids:
                for checker_id in checkers_id:
                    observation = observations[camera_id][checker_id]
                    for point in observation._2d:
                        if not np.isnan(point).any():
                            ax.text(point[0], point[1], checker_id, color='black', fontsize=10, alpha=1)
                            break
                # ax.text(observation._2d[0, 0], observation._2d[0, 1], checker_id, color='black', fontsize=10, alpha=1)

                



            if which == "both" or which == "conform":
                for checker_id, _2d in _2d_observations_conform.items(): 
                    color = 'blue'
                    alpha = 1
                    marker = MARKERS_OBSERVATIONS[SceneType.SYNTHETIC]
                    if np.isnan(_2d).any():
                        print(f"NaN values found in observations for checker_id {checker_id} in camera_id {camera_id}")
                    ax.plot(_2d[:, 0], _2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha)  
                    
                for checker_id, _2d in _2d_reprojections_conform.items(): 
                    color = 'blue'
                    alpha = 1
                    marker = MARKERS_OBSERVATIONS[SceneType.ESTIMATE]
                    if np.isnan(_2d).any():
                        print(f"NaN values found in observations for checker_id {checker_id} in camera_id {camera_id}")
                    ax.plot(_2d[:, 0], _2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha) 

            if which == "both" or which == "non-conform":
                for checker_id, _2d in _2d_observations_nonconform.items(): 
                    color = 'red'
                    alpha = 1
                    marker = MARKERS_OBSERVATIONS[SceneType.SYNTHETIC]
                    if np.isnan(_2d).any():
                        print(f"NaN values found in observations for checker_id {checker_id} in camera_id {camera_id}")
                    ax.plot(_2d[:, 0], _2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha)  
                    
                for checker_id, _2d in _2d_reprojections_nonconform.items(): 
                    color = 'red'
                    alpha = 1
                    marker = MARKERS_OBSERVATIONS[SceneType.ESTIMATE]
                    if np.isnan(_2d).any():
                        print(f"NaN values found in observations for checker_id {checker_id} in camera_id {camera_id}")
                    ax.plot(_2d[:, 0], _2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha)  

                for checker_id, _2d in _2d_observations_nonconform_by_discard_of_chessboard.items(): 
                    color = 'orange'
                    alpha = 1
                    marker = MARKERS_OBSERVATIONS[SceneType.SYNTHETIC]
                    if np.isnan(_2d).any():
                        print(f"NaN values found in observations for checker_id {checker_id} in camera_id {camera_id}")
                    ax.plot(_2d[:, 0], _2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha)  
                    
                for checker_id, _2d in _2d_reprojections_nonconform_by_discard_of_chessboard.items(): 
                    color = 'orange'
                    alpha = 1
                    marker = MARKERS_OBSERVATIONS[SceneType.ESTIMATE]
                    if np.isnan(_2d).any():
                        print(f"NaN values found in observations for checker_id {checker_id} in camera_id {camera_id}")
                    ax.plot(_2d[:, 0], _2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha)  
                            
                # _2d_observations_conform.append
            # for scene in scenes: 
            #     # reprojections_errors = reprojection_error_per_view(scene, observations)

            #     plot_reprojections_in_camera(scene.reprojections,  
            #                                  camera_id=camera_id, 
            #                                  ax=ax)
            # if observations:
            #     plot_observations_in_camera(observations, 
            #                                 camera_id=camera_id, 
            #                                 ax=ax)

        for i, ax in enumerate(axs): 
            if not axs_used[i]: 
                ax.axis('off')  # Turn off unused subplots

        if save_fig:
            # Create the directory if it does not exist
            os.makedirs(os.path.dirname(save_path), exist_ok=True)
            fig.savefig(save_path, dpi=300, bbox_inches='tight')
        if show_fig:
            plt.show(block=False)
        
        # plt.show(block=False)

def plot_reprojections_in_camera(correspondences: Correspondences, 
                                 camera_id: idtype,
                                 ax) -> None:
    # errors_list = []
    # for errors_of_camera in reprojections_errors.values():
    #     errors_list.extend(errors_of_camera.values())

    # error_min = min(errors_list)
    # error_max = max(errors_list)

    for checker_id in correspondences[camera_id]:
        observation = correspondences[camera_id][checker_id]
        if observation is not None and observation._is_conform:
            _2d_masked = np.zeros_like(observation._2d)
            _2d_masked[:,:] = observation._2d[:,:]
            _2d_masked[~observation._conformity_mask,:] = np.nan
        # error = reprojections_errors[camera_id].get(checker_id)
        # if error:
        #     marker = MARKERS_OBSERVATIONS[SceneType.ESTIMATE]
        #     if cmap == "id":
        #         color = get_color_from_id(checker_id)
        #     elif cmap == "reprojection_error": 
        #         color = get_color_from_error(error, error_min, error_max)
        #     else:
        #         raise ValueError("cmap not implemented.")
        color = 'blue'
        alpha = 1
        marker = MARKERS_OBSERVATIONS[SceneType.ESTIMATE]
        ax.plot(observation._2d[:, 0], observation._2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha)  
        ax.text(observation._2d[0, 0], observation._2d[0, 1], checker_id, color='black', fontsize=10, alpha=1)

    # if cmap == "reprojection_error": 
    #     sm = plt.cm.ScalarMappable(cmap=plt.get_cmap('coolwarm'), norm=plt.Normalize(vmin=error_min, vmax=error_max))
    #     sm.set_array([])

    #     # Add the colorbar to the figure
    #     cbar = plt.colorbar(sm, ax=ax)
    #     cbar.set_label('Reprojection Error')

def plot_observations_in_camera(correspondences: Correspondences, 
                                camera_id: idtype,
                                ax, 
                                cmap=None, 
                                reprojections_errors=None) -> None:
    for checker_id in correspondences[camera_id]:
        observation = correspondences[camera_id][checker_id]
        if observation and observation._is_conform:
            marker = MARKERS_OBSERVATIONS[SceneType.SYNTHETIC]
            alpha = 0.5
            color = "black"
    
            ax.plot(observation._2d[:, 0], observation._2d[:, 1], marker, markersize=4, markeredgewidth=1, color=color, alpha=alpha)  
            ax.text(observation._2d[0, 0], observation._2d[0, 1], checker_id, color=color, fontsize=10, alpha=1)

def latex_escape_underscores(text: str) -> str:
    """Replace underscores with escaped underscores for LaTeX."""
    return text.replace("_", r"\_")

def plot_scene(scene: SceneCheckerboard, show_ids=False) -> None:
    """
    Plots each camera and checker in the scene, properly escaping underscores
    so LaTeX doesn't interpret them as multiple subscripts.
    """
    mod = MODIFIERS[scene.type]

    for camera in scene.cameras.values():
        escaped_cam_id = latex_escape_underscores(str(camera.id))
        # Build something like: $\{\hat{C}_{zed\_x\_m}\}$
        # We do this by concatenating normal strings:
        #   "$\\{" ->  `$\{`
        #   + mod -> e.g. `\hat`
        #   + "{C}_{" -> `{C}_{`
        #   + escaped_cam_id -> e.g. `zed\_x\_m`
        #   + "}\\}$" -> `}\}` (literal closing brace) + `$`
        name = "$\\{" + mod + "{C}_{" + escaped_cam_id + "}\\}$"
        camera.plot(name)

    for checker in scene.checkers.values():
        if show_ids:
            escaped_checker_id = latex_escape_underscores(str(checker.id))
            # Similarly: $\{\hat{B}_{checker\_id}\}$
            name = "$\\{" + mod + "{B}_{" + escaped_checker_id + "}\\}$"
        else:
            name = ""
        checker.plot(name)


# def plot_reprojection_errors(scene_estimate: SceneCheckerboard, 
#                              observations: Correspondences, 
#                              show_fig = True,
#                             save_fig = False, 
#                             save_path = None) -> None: 
    
#     n = len(scene_estimate.cameras)
#     cols = math.ceil(math.sqrt(n))
#     rows = math.ceil(n/ cols)

#     fig_width = cols * 4  # Each subplot has a width of 4 inches (adjust as needed)
#     fig_height = rows * 4  # Each subplot has a height of 3 inches (adjust as needed)

#     # Create a figure with dynamic size
#     fig, axs = plt.subplots(rows, cols, figsize=(fig_width, fig_height))
    
#     # fig, axs = plt.subplots(rows, cols)
#     axs = np.array(axs).reshape(-1) if n > 1 else [axs]
#     axsUsed = [False for ax in axs]

#     axIndex = 0
    
#     all_errors = []
#     all_errors_conform = []

#     for camera in scene_estimate.cameras.values(): 
#         ax = axs[axIndex]
#         axsUsed[axIndex] = True
#         axIndex += 1
        
#         # errors_conform = get_errors_x_y_in_cam(camera, scene_estimate.object_points, observations, which="conform")
#         # errors_nonconform = get_errors_x_y_in_cam(camera, scene_estimate.object_points, observations, which="non-conform")
#         errors_conform = get_errors_x_y_in_cam(camera, scene_estimate.checkers, observations, which="conform")
#         errors_nonconform = get_errors_x_y_in_cam(camera, scene_estimate.checkers, observations, which="non-conform")
#         # print(errors_conform)
#         # print(errors_nonconform)
#         # errors_conform = list(errors_conform.values())
#         # errors_nonconform = list(errors_nonconform.values())
#         errors_tot = np.concatenate(errors_conform+errors_nonconform, axis=0)        # plt.xlim(-plot_lim, plot_lim)

        
#         all_errors.extend(errors_tot)
#         all_errors_conform.extend(errors_conform)
        
#         # error_min = np.nanmin(errors_tot)
#         # error_max = np.nanmax(errors_tot)

#         # plot_lim = max(abs(error_min), abs(error_max))

#         # # fig = plt.figure()
#         # # plt.xlim(-plot_lim, plot_lim)
#         # # plt.ylim(-plot_lim, plot_lim)
#         # ax.set_xlim([-plot_lim, plot_lim])
#         # ax.set_ylim([-plot_lim, plot_lim])
#         # ax = plt.gca()
#         aspect_ratio = 1
#         ax.set_aspect(aspect_ratio / ax.get_data_ratio())


#         for errors_ in errors_conform: 
#             ax.scatter(errors_[:,0], errors_[:,1], s=0.5, c='blue', marker = 'o', alpha=1)  

#         for errors_ in errors_nonconform: 
#             ax.scatter(errors_[:,0], errors_[:,1], s=0.5, c='red', marker = 'o', alpha=1)  
      
#         circle = plt.Circle((0,0), 1, edgecolor='red', facecolor='none', linewidth=1.5, alpha = 1)
#         ax.add_patch(circle)

#         circle = plt.Circle((0,0), 0.7, edgecolor='red', facecolor='none', linewidth=1.5, alpha = 1)
#         ax.add_patch(circle)

#         title = f"Repr. errors in {camera.id}"
#         ax.set_title(title)
#         # plt.title(title)
    
#     # compute axis limits (same for all subplots)
#     # error_min = np.nanmin(all_errors)
#     # error_max = np.nanmax(all_errors)
#     # plot_lim = max(abs(error_min), abs(error_max))
#     error_min = np.nanmin(all_errors_conform)
#     error_max = np.nanmax(all_errors_conform)
#     # plot_lim = max(abs(error_min), abs(error_max))
#     plot_lim = 2
#     axIndex = 0
#     for camera in scene_estimate.cameras.values(): 
#         ax = axs[axIndex]
#         axIndex += 1
#         ax.set_xlim([-plot_lim, plot_lim])
#         ax.set_ylim([-plot_lim, plot_lim])

#         if save_fig:
#             # Create the directory if it does not exist
#             os.makedirs(os.path.dirname(save_path), exist_ok=True)
#             fig.savefig(save_path, dpi=300, bbox_inches='tight')
#         if show_fig:
#             plt.show(block=False)
            


# def plot_reprojection_errors(scene_estimate: SceneCheckerboard, 
#                              observations: Correspondences) -> None: 
    
#     for camera in scene_estimate.cameras.values(): 
#         errors_conform = get_errors_x_y_in_cam(camera, scene_estimate.checkers, observations, which="conform")
#         errors_nonconform = get_errors_x_y_in_cam(camera, scene_estimate.checkers, observations, which="non-conform")
#         errors_tot = np.concatenate(errors_conform+errors_nonconform, axis=0)        # plt.xlim(-plot_lim, plot_lim)

#         error_min = np.nanmin(errors_tot)
#         error_max = np.nanmax(errors_tot)

#         plot_lim = max(abs(error_min), abs(error_max))

#         fig = plt.figure()
#         plt.xlim(-plot_lim, plot_lim)
#         plt.ylim(-plot_lim, plot_lim)
#         ax = plt.gca()
#         aspect_ratio = 1
#         ax.set_aspect(aspect_ratio / ax.get_data_ratio())


#         for errors_ in errors_conform: 
#             plt.plot(errors_[:,0], errors_[:,1], 'o', markersize=1, markeredgewidth=1, color='blue', alpha=1)  

#         for errors_ in errors_nonconform: 
#             plt.plot(errors_[:,0], errors_[:,1], 'o', markersize=1, markeredgewidth=1, color='red', alpha=1)  
      
#         circle = plt.Circle((0,0), 0.5, edgecolor='red', facecolor='none', linewidth=1.5, alpha = 1)
#         ax.add_patch(circle)

#         circle = plt.Circle((0,0), 0.75, edgecolor='red', facecolor='none', linewidth=1.5, alpha = 1)
#         ax.add_patch(circle)

#         title = f"Errors in cam {camera.id}"
#         plt.title(title)

#     return 

