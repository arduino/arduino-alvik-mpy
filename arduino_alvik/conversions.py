# MEASUREMENT UNITS CONVERSION #

from math import pi


def conversion_method(func):
    def wrapper(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except KeyError:
            raise ConversionError(f'Cannot {func.__name__} from {args[1]} to {args[2]}')
        except TypeError:
            return None
        except Exception as e:
            raise ConversionError(f'Unexpected error: {e}')
    return wrapper


@conversion_method
def convert_rotational_speed(value: float, from_unit: str, to_unit: str) -> float:
    """
    Converts a rotational speed value from one unit to another
    :param value:
    :param from_unit: unit of input value
    :param to_unit: unit of output value
    :return:
    """
    speeds = {'rpm': 1.0, 'deg/s': 1/6, 'rad/s': 60/(2*pi), 'rev/s': 60}
    return value * speeds[from_unit.lower()] / speeds[to_unit.lower()]


@conversion_method
def convert_angle(value: float, from_unit: str, to_unit: str) -> float:
    """
    Converts an angle value from one unit to another
    :param value:
    :param from_unit: unit of input value
    :param to_unit: unit of output value
    :return:
    """
    angles = {'deg': 1.0, 'rad': 180/pi, 'rev': 360, 'revolution': 360, '%': 3.6, 'perc': 3.6}
    return value * angles[from_unit.lower()] / angles[to_unit.lower()]


@conversion_method
def convert_distance(value: float, from_unit: str, to_unit: str) -> float:
    """
    Converts a distance value from one unit to another
    :param value:
    :param from_unit: unit of input value
    :param to_unit: unit of output value
    :return:
    """
    distances = {'cm': 1.0, 'mm': 0.1, 'm': 100, 'inch': 2.54, 'in': 2.54}
    return value * distances[from_unit.lower()] / distances[to_unit.lower()]


@conversion_method
def convert_speed(value: float, from_unit: str, to_unit: str) -> float:
    """
    Converts a distance value from one unit to another
    :param value:
    :param from_unit: unit of input value
    :param to_unit: unit of output value
    :return:
    """
    distances = {'cm/s': 1.0, 'mm/s': 0.1, 'm/s': 100, 'inch/s': 2.54, 'in/s': 2.54}
    return value * distances[from_unit.lower()] / distances[to_unit.lower()]


class ConversionError(Exception):
    pass
