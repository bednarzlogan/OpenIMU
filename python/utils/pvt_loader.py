from typing import Dict, List
import numpy as np
from dataclasses import dataclass
from pyproj import CRS, Transformer
import datetime

"""
This script is responsible for loading pvt data from GNSS SDR and converting it to 
ENU data for ingestion into the UKF/sensor fusion schemes
"""

@dataclass
class SearchInfo:
    # tell us what the user is seaching for and how to organize
    # pulled data

    # search params
    lat_signal: str
    lon_signal: str
    height_signal: str
    heading_signal: str
    velocity_signal: str

    lat_cov_signal: str
    lon_cov_signal: str
    height_cov_signal: str
    heading_cov_signal: str
    velocity_cov_signal: str

    def pack_search_info(self) -> Dict[str, str]:
        # pack the search info into a dictionary
        return {
            "lat_signal": self.lat_signal,
            "lon_signal": self.lon_signal,
            "height_signal": self.height_signal,
            "heading_signal": self.heading_signal,
            "velocity_signal": self.velocity_signal,
            "lat_cov_signal": self.lat_cov_signal,
            "lon_cov_signal": self.lon_cov_signal,
            "height_cov_signal": self.height_cov_signal,
            "heading_cov_signal": self.heading_cov_signal,
            "velocity_cov_signal": self.velocity_cov_signal
        }


class PathInfo:
    # use the search info to return a matching struct
    lats: List[float]
    lons: List[float]
    heights: List[float]
    headings: List[float]
    velocities: List[float]

    lat_vars: List[float]
    lon_vars: List[float]
    height_vars: List[float]
    heading_vars: List[float]
    velocity_vars: List[float]

    # store ENU positions and variances
    enu_positions: List[tuple]
    enu_variances: List[tuple]

    def __init__(self) -> None:
        # initialize the lists to empty
        # these will be filled with data as we parse the file
        # we don't want to have to check if these are None when we append data
        self.lats = []
        self.lons = []
        self.heights = []
        self.headings = []
        self.velocities = []

        self.lat_vars = []
        self.lon_vars = []
        self.height_vars = []
        self.heading_vars = []
        self.velocity_vars = []

        self.enu_positions = []
        self.enu_variances = []
        self.enu_velocities = []


def convert_to_enu(path_info: PathInfo) -> None:
    # Define reference point from first lat/lon/height
    origin_lat = path_info.lats[0]
    origin_lon = path_info.lons[0]
    origin_alt = path_info.heights[0]

    ecef = Transformer.from_crs("EPSG:4979", "EPSG:4978", always_xy=True)

    # local ENU-like CRS
    enu_crs = CRS.from_proj4(
        f"+proj=aeqd +lat_0={origin_lat} +lon_0={origin_lon} +h_0={origin_alt} +datum=WGS84 +units=m +no_defs"
    )
    enu_transformer = Transformer.from_crs("EPSG:4978", enu_crs, always_xy=True)

    path_info.enu_positions = []
    path_info.enu_variances = []

    for lat, lon, alt, heading, velocity, lat_var_deg, lon_var_deg, height_var_m2, velocity_var in zip(
        path_info.lats, path_info.lons, path_info.heights, path_info.headings, path_info.velocities,
        path_info.lat_vars, path_info.lon_vars, path_info.height_vars, path_info.velocity_vars
    ):
        # convert positions
        x, y, z = ecef.transform(lon, lat, alt)
        e, n, u = enu_transformer.transform(x, y, z)
        path_info.enu_positions.append((e, n, u))

        # resolve velocities into ENU frame
        vx, vy = velocity * np.cos(heading), velocity * np.sin(heading)
        path_info.enu_velocities.append((vx, vy, 0.0))  # z-component is zero for horizontal velocity

        # convert variances from degrees to meters**2
        lat_m_std = 111320.0 * np.sqrt(lat_var_deg)  # 1 degree ~ 111.32 km
        lon_m_std = 111320.0 * np.cos(np.radians(lat)) * np.sqrt(lon_var_deg)
        path_info.enu_variances.append((
            lat_m_std**2,
            lon_m_std**2,
            height_var_m2,
            np.cos(heading) * velocity_var,  # vairance expressed in the right coordinate system
            np.sin(heading) * velocity_var,
            0.5**2 # inject some uncertainty in the z-component velocity
        ))

def write_out_waypoints(path_info: PathInfo, output_file: str) -> None:
    with open(output_file, 'w') as file:
        # write the header
        file.write("e,n,vx,vy,vz,cov_e,cov_n,cov_u,cov_vx,cov_vy,cov_vz\n")

        for enu in zip(path_info.enu_positions, path_info.enu_velocities, path_info.enu_variances):
            e, n, u = enu[0]
            vx, vy, vz = enu[1]
            cov_e, cov_n, cov_u, cov_vx, cov_vy, cov_vz = enu[2]
            file.write(f"{e},{n},{u},{vx},{vy},{vz},{cov_e},{cov_n},{cov_u},{cov_vx},{cov_vy},{cov_vz}\n")

def load_data(path_to_data_file: str, 
              delimiter: str, 
              search_info: SearchInfo) -> PathInfo:
    # allocate the return struct 
    path_info = PathInfo()

    # load the data
    with open(path_to_data_file) as file:
        # parse line-by-line, extracting tokens
        header_line = file.readline().split(delimiter)
        expected_number_columns: int = len(header_line) # catch malformed lines

        # find the indices of the desired info
        def get_indx(key: str) -> int:
            search_indx: int = next((ind for ind, sig 
                                     in enumerate(header_line)
                                     if key in sig), -1)
            return search_indx
        
        # get packed search info for index location
        indices_of_interest: List[int] = [get_indx(signal)
                                          for signal in search_info.pack_search_info().values()]
        
        # check if all indices were found
        if any(ind < 0 for ind in indices_of_interest):
            raise ValueError("Not all signals were found in the data file header.")
        
        # allocate return struct
        return_raw_data = [[] for _ in range(len(indices_of_interest))]
        
        for line in file:
            tokens: List[str] = line.split(delimiter)
            if len(tokens) != expected_number_columns:
                print(f"Warning: Malformed line skipped (expected {expected_number_columns} columns, got {len(tokens)}): {line.strip()}")
                continue
            # extract the data for the indices of interest
            for i, ind in enumerate(indices_of_interest):
                try:
                    return_raw_data[i].append(float(tokens[ind]))
                except ValueError:
                    print(f"Warning: Non-numeric data found in column {ind} for line: {line.strip()}")
                    return_raw_data[i].append(float('nan'))

        # unpack the data into the path_info struct
        path_info.lats = return_raw_data[0]
        path_info.lons = return_raw_data[1]
        path_info.heights = return_raw_data[2]
        path_info.headings = return_raw_data[3]
        path_info.velocities = return_raw_data[4]
        path_info.lat_vars = return_raw_data[5]
        path_info.lon_vars = return_raw_data[6]
        path_info.height_vars = return_raw_data[7]
        path_info.heading_vars = return_raw_data[8]
        path_info.velocity_vars = return_raw_data[9]

    return path_info


def main(path_to_data: str) -> None:
    # define what we want to pull from the data file]
    search = SearchInfo(lat_signal = "Latitude (GT Lat) [deg]",
                        lon_signal = "Longitude (GT Lon) [deg]",
                        height_signal = "Height above ellipsoid (GT Height) [m]",
                        heading_signal = "Heading (0 [deg] = East, counterclockwise) - (GT Heading) [rad]",
                        velocity_signal = "Velocity (GT Velocity) [m/s]",
                        lat_cov_signal = "Latitude Cov (GT Lat) [deg]",
                        lon_cov_signal = "Longitude Cov (GT Lon) [deg]",
                        height_cov_signal = "Height above ellipsoid Cov (GT Height) [m]",
                        heading_cov_signal = "Heading Cov (0 [deg] = East, counterclockwise) - (GT Heading) [rad]",
                        velocity_cov_signal = "Velocity Cov (GT Velocity) [m/s]")
    
    # load the data into the path_info struct
    path_info = load_data(path_to_data, 
                          delimiter=";", 
                          search_info=search)
    
    # update the path_info with ENU positions and variances
    convert_to_enu(path_info)

    # write out waypoint data into a file for the simulator
    filename = f"waypoints_{datetime.datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    write_out_waypoints(path_info, filename)


if __name__ == "__main__":
    path_to_data = "berlin1_potsdamer_platz\RXM-RAWX.csv"
    main(path_to_data)
