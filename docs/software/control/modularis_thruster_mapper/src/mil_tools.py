from typing import List, Tuple, Callable, Any, Optional

import geometry_msgs.msg as geometry_msgs
import nav_msgs.msg as nav_msgs 
import numpy as np
import rclpy
from rclpy.node import Node
import time
import std_msgs.msg as std_msgs
import tf_transformations as transformations
from threading import Lock

def rosmsg_to_numpy(rosmsg, keys=None):
    """
    Convert an arbitrary ROS msg to a numpy array
    With no additional arguments, it will by default handle:
        Point2D, Point3D, Vector3D, Quaternion and any lists of these (like Polygon)

    Ex:
        quat = Quaternion(1.0, 0.0, 0.0, 0.0)
        quat is not a vector, you have quat.x, quat.y,... and you can't do math on that

        But wait, there's hope!
            rosmsg_to_numpy(quat) -> array([1.0, 0.0, 0.0, 0.0])

        Yielding a vector, which you can do math on!

        Further, imagine a bounding box message, BB, with properties BB.x, BB.h, BB.y, and BB.w

            rosmsg_to_numpy(BB, ['x', 'h', 'y', 'w']) -> array([BB.x, BB.h, BB.y, BB.w])

            or...
            rosmsg_to_numpy(some_Pose2D, ['x', 'y', 'yaw']) = array([x, y, yaw])

    Note:
        - This function is designed to handle the most common use cases (vectors, points and quaternions)
            without requiring any additional arguments.
    """

    # Recurse for lists like geometry_msgs/Polygon, Pointclou
    if isinstance(rosmsg, list):
        output_array = []
        for item in rosmsg:
            output_array.append(rosmsg_to_numpy(item, keys=keys))
        return np.array(output_array).astype(np.float32)

    if keys is None:
        keys = ["x", "y", "z", "w"]
        output_array = []
        for key in keys:
            # This is not necessarily the fastest way to do this
            if hasattr(rosmsg, key):
                output_array.append(getattr(rosmsg, key))
            else:
                break
        assert (
            len(output_array) != 0
        ), f"Input type {type(rosmsg).__name__} has none of these attributes {keys}."
        return np.array(output_array).astype(np.float32)

    else:
        output_array = np.zeros(len(keys), np.float32)
        for n, key in enumerate(keys):
            output_array[n] = getattr(rosmsg, key)

        return output_array


def pose_to_numpy(pose: geometry_msgs.Pose) -> Tuple[float, float]:
    """
    Turns a :class:`Pose` message into a tuple of position and orientation.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        pose (Pose): The pose message.

    Returns:
        Tuple[float, float]: The tuple of position and orientation.
    """
    # TODO Add unit tests
    position = rosmsg_to_numpy(pose.position)
    orientation = rosmsg_to_numpy(pose.orientation)
    return position, orientation



def twist_to_numpy(twist: geometry_msgs.Twist):
    """
    Turns a :class:`Twist` message into a tuple of linear and angular speeds.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        twist (Twist): The twist message.

    Returns:
        Tuple[List[float], List[float]]: The tuple of linear and angular speeds.
    """
    # TODO Add unit tests
    linear = rosmsg_to_numpy(twist.linear)
    angular = rosmsg_to_numpy(twist.angular)
    return linear, angular


def odometry_to_numpy(odom: nav_msgs.Odometry):
    """
    Turns a :class:`Odometry` message into its pose, twist, pose covariance,
    and twist covariance.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        odom (Odometry): The odometry message.

    Returns:
        Tuple[Tuple[np.ndarray, np.ndarray], Tuple[np.ndarray, np.ndarray],
            np.ndarray, np.ndarray]: The tuple of pose, twist, pose covariance,
            and twist covariance.
    """
    # TODO Add unit tests
    pose = pose_to_numpy(odom.pose.pose)
    pose_covariance = np.array(odom.pose.covariance).reshape(6, 6)

    twist = twist_to_numpy(odom.twist.twist)
    twist_covariance = np.array(odom.twist.covariance).reshape(6, 6)

    return pose, twist, pose_covariance, twist_covariance


def wrench_to_numpy(wrench: geometry_msgs.Wrench):
    """
    Turns a :class:`Wrench` message into its force and torque, represented as
    numpy arrays.

    .. warning ::

        This method relies on a method (:meth:`mil_ros_tools.rosmsg_to_numpy`)
        which will be deprecated in the future. This method will need to be updated.

    Args:
        wrench (Wrench): The wrench message.

    Returns:
        Tuple[np.ndarray, np.ndarray]: The tuple of force and torque.
    """
    force = rosmsg_to_numpy(wrench.force)
    torque = rosmsg_to_numpy(wrench.torque)
    return force, torque



def numpy_to_point(vector: np.ndarray) -> geometry_msgs.Point:
    """
    Turns a List[:class:`float`] into a :class:`Point` message.

    Args:
        vector (np.ndarray): The vector to convert

    Returns:
        geometry_msgs.Point: The constructed message.
    """
    np_vector = np.array(vector)
    if np_vector.size == 2:
        np_vector = np.append(np_vector, 0)  # Assume user is give 2d point

    return geometry_msgs.Point(*np_vector)


def numpy_to_quaternion(np_quaternion: np.ndarray) -> geometry_msgs.Quaternion:
    """
    Turns a List[:class:`float`] into a :class:`Quaternion` message.

    Args:
        np_quaternion (np.ndarray): The vector to convert. Should have four values,
            representing ``x``, ``y``, ``z``, and ``w``.

    Returns:
        Quaternion: The constructed message.
    """
    return geometry_msgs.Quaternion(*np_quaternion)


def numpy_to_twist(
    linear_vel: np.ndarray,
    angular_vel: np.ndarray,
) -> geometry_msgs.Twist:
    """
    Turns two :class:`np.ndarray`s into a :class:`Twist` message.

    Args:
        linear_vel (np.ndarray): The vector to convert. Values should represent
            the individual components of ``x``, ``y``, and ``z``.
        angular_vel (np.ndarray): The vector to convert. Values should represent
            the individual components of ``x``, ``y``, and ``z``.

    Returns:
        Twist: The constructed message.
    """
    # TODO Add unit tests
    return geometry_msgs.Twist(
        linear=geometry_msgs.Vector3(*linear_vel),
        angular=geometry_msgs.Vector3(*angular_vel),
    )


def numpy_to_wrench(forcetorque: np.ndarray):
    """
    Turns a np.ndarray into a :class:`Wrench` message.

    Args:
        forcetorque (np.ndarray): The vector to convert. Values should represent
            the individual components of ``force.x``, ``force.y``, ``force.z``,
            ``torque.x``, ``torque.y``, and ``torque.z``.

    Returns:
        Wrench: The constructed message.
    """
    return geometry_msgs.Wrench(
        force=geometry_msgs.Vector3(*forcetorque[:3]),
        torque=geometry_msgs.Vector3(*forcetorque[3:]),
    )



def numpy_matrix_to_quaternion(np_matrix: np.ndarray):
    """
    Given a 3x3 rotation matrix, convert to a quaternion, and return the quaternion
    as a ROS message.

    Args:
        np_matrix (np.ndarray): The 3x3 rotation matrix.

    Returns:
        Quaternion: The constructed message.
    """
    assert np_matrix.shape == (3, 3), "Must submit 3x3 rotation matrix"
    pose_mat = np.eye(4)
    pose_mat[:3, :3] = np_matrix
    np_quaternion = transformations.quaternion_from_matrix(pose_mat)
    return geometry_msgs.Quaternion(*np_quaternion)


def numpy_pair_to_pose(
    np_translation: np.ndarray,
    np_rotation_matrix: np.ndarray,
) -> geometry_msgs.Pose:
    """
    Convert a rotation matrix and point pair to a Pose message.

    Args:
        np_translation (np.ndarray): An array of 2 or 3 points representing the
            object's position.
        np_rotation_matrix (np.ndarray): A 3x3 rotation matrix.

    Returns:
        geometry_msgs.Pose: The Pose message with the position and orientation.
    """
    orientation = numpy_matrix_to_quaternion(np_rotation_matrix)
    position = numpy_to_point(np_translation)
    return geometry_msgs.Pose(position=position, orientation=orientation)



def numpy_quat_pair_to_pose(
    np_translation: np.ndarray,
    np_quaternion: np.ndarray,
) -> geometry_msgs.Pose:
    """
    Convert a quaternion array and point array pair to a Pose message.

    Args:
        np_translation (np.ndarray): An array of 2 or 3 points representing the
            object's position.
        np_quaternion (np.ndarray): An array with 4 points, representing ``x``,
            ``y``, ``z``, and ``w`` of a quaternion describing the object.

    Returns:
        geometry_msgs.Pose: The Pose message with the position and orientation.
    """
    orientation = numpy_to_quaternion(np_quaternion)
    position = numpy_to_point(np_translation)
    return geometry_msgs.Pose(position=position, orientation=orientation)


def numpy_to_points(points: List[np.ndarray]) -> List[geometry_msgs.Point]:
    """
    Convert a list of :class:`np.ndarray`s into :class:`Point` messages.

    Args:
        points (List[np.ndarray]): The points that will be converted to messages.

    Returns:
        List[Point]: The resulting point messages.
    """
    ret = []
    for point in points:
        ret.append(numpy_to_point(point))
    return ret



def numpy_to_polygon(polygon: List[np.ndarray]) -> geometry_msgs.Polygon:
    """
    Convert a list of :class:`np.ndarray`s (representing :class:`Point`s) into a
    :class:`Polygon` message.

    Args:
        points (List[np.ndarray]): The points that will be added to the polygon.

    Returns:
        Polygon: The resulting message.
    """
    points = numpy_to_points(polygon)
    return geometry_msgs.Polygon(points=points)


def numpy_to_vector3(vec: np.ndarray) -> geometry_msgs.Vector3:
    """
    Convert a :class:`np.ndarray` of :class:`float`s with length 3 into a :class:`Vector3`
    message.

    Args:
        vec (np.ndarray): A numpy array with ``x``, ``y``, and ``z``.

    Returns:
        Vector3: The Vector3 message.
    """
    assert len(vec) == 3
    return geometry_msgs.Vector3(*vec)


def numpy_to_colorRGBA(color: np.ndarray) -> std_msgs.ColorRGBA:
    """
    Convert a :class:`np.ndarray` of :class:`float`s with length 4 into a :class:`ColorRGBA`
    message.

    Args:
        color (np.ndarray): A numpy array with ``r``, ``g``, ``b``, and ``a``.

    Returns:
        ColorRGBA: The ColorRGBA message.
    """
    return std_msgs.ColorRGBA(*color)

def make_header(frame="/body", stamp=None) -> std_msgs.Header:
    """
    Creates a message header.

    Args:
        frame (str): The name of the frame to attach to the header. Defaults to
            ``/body``.
        stamp (Optional[rclpy.time.Time]): The timestamp to attach to the header
            of the message. If ``None``, then the current time is used.

    Returns:
        Header: The constructed header.
    """
    if stamp is None:
        stamp = rclpy.clock.Clock().now().to_msg()

    header = std_msgs.Header(stamp=stamp, frame_id=frame)
    return header



def make_wrench_stamped(force: List[float], torque: List[float], frame: str = "/body"):
    """
    Makes a WrenchStamped message.

    Args:
        force (List[float]): An array representing the force components. The list
            should contain ``force.x``, ``force.y``, ``force.z``.
        torque (List[float]): An array representing the torque components. The list
            should contain ``torque.x``, ``torque.y``, and ``torque.z``.
        frame (str): The frame to attach to the header. Defaults to ``/body``.

    Returns:
        WrenchStamped: The constructed message.
    """
    wrench = geometry_msgs.WrenchStamped(
        header=make_header(frame),
        wrench=geometry_msgs.Wrench(
            force=geometry_msgs.Vector3(*force),
            torque=geometry_msgs.Vector3(*torque),
        ),
    )
    return wrench


def make_pose_stamped(
    position: List[float],
    orientation: List[float],
    frame: str = "/body",
) -> geometry_msgs.WrenchStamped:
    """
    Makes a PoseStamped message.

    Args:
        position (List[float]): An array representing the force components. The list
            should contain ``position.x``, ``position.y``, ``position.z``.
        orientation (List[float]): An array representing the torque components. The list
            should contain ``orientation.x``, ``orientation.y``, ``orientation.z``,
            and ``orientation.w``.
        frame (str): The frame to attach to the header. Defaults to ``/body``.

    Returns:
        WrenchStamped: The constructed message.
    """
    wrench = geometry_msgs.WrenchStamped(
        header=make_header(frame),
        pose=geometry_msgs.Pose(
            force=geometry_msgs.Vector3(*position),
            orientation=geometry_msgs.Quaternion(*orientation),
        ),
    )
    return wrench

def ros_to_np_3D(msg):
    xyz_array = np.array([msg.x, msg.y, msg.z])
    return xyz_array

def quat_to_rotvec(quat: List[float]) -> np.ndarray:
    """
    Convert a quaternion to a rotation vector.

    Args:
        quat (List[float]): An array representing the quaternion.

    Returns:
        np.ndarray: The new rotation vector.
    """
    # For unit quaternion, return 0 0 0
    if np.all(np.isclose(quat[0:3], 0)):
        return np.array([0.0, 0.0, 0.0])
    if quat[3] < 0:
        quat = -quat
    quat = transformations.unit_vector(quat)
    angle = np.arccos(quat[3]) * 2
    norm = np.linalg.norm(quat)
    axis = quat[0:3] / norm
    return axis * angle

def quaternion_matrix(q):
    mat_h = transformations.quaternion_matrix(q)
    return mat_h[:3, :3] / mat_h[3, 3]
    
def thread_lock(lock: Lock):
    """
    A decortor for using an existing thread lock to thread-lock a function.
    This prevents the function from being executed by multiple threads at once.

    .. code-block:: python3

        import threading
        lock = threading.Lock()

        @thread_lock(lock)
        def my_function(a, b, c):
            print(a, b, c)
    """

    def lock_thread(function_to_lock: Callable):
        """
        thread_lock(function) -> locked function
        Thread locking decorator
            If you use this as a decorator for a function, it will apply a threading lock during
            the execution of that function, which guarantees that no ROS callbacks can change the
            state of data while it is executing. This is critical to make sure that a new message
            being sent doesn't cause a weird serial interruption
        """

        def locked_function(args, **kwargs):
            # Get threading lock
            with lock:
                result = function_to_lock(args, **kwargs)
            # Return, pretending the function hasn't changed at all
            return result

        # Return the function with locking added
        return locked_function

    return lock_thread

def wait_for_param(
    param_name: str,
    timeout: Optional[float] = None,
    poll_rate: float = 0.1,
) -> Optional[Any]:
    """
    Blocking wait for a parameter named to exist. Polls at the frequency of poll_rate.
    Once the parameter exists, return get and return it.

    This function intentionally leaves failure logging duties to the developer.

    Args:
        node (Node): The ROS 2 node instance.
        param_name (str): The name of the parameter to watch.
        timeout (Optional[float]): The number of seconds to wait for the
            parameter to exist.
        poll_rate (float): The Hz rate to poll at.

    Returns:
        Optional[Any]: If found, the value of the parameter. Returns ``None`` if
        the parameter never came to exist.
    """
    start_time = time.monotonic()
    while rclpy.ok():
        # Check if the parameter now exists
        if Node.has_parameter(param_name):
            return Node.get_parameter(param_name).get_parameter_value().value

        # If we exceed a defined timeout, return None
        if timeout is not None and time.monotonic() - start_time > timeout:
            return None

        # Continue to poll at poll_rate
        time.sleep(1.0 / poll_rate)

