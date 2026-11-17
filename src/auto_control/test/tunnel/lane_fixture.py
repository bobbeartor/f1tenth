from types import SimpleNamespace as NS


def ns(seconds):
    return round(seconds * 1e9)


def lane(seconds, lateral=0., slope=0., valid=True, start=.10, count=50, step=.02,
         padding=30):
    stamp = ns(seconds)
    points = [NS(x=(.6 - (lateral + slope * (start + i * step))) / .01 + padding - .5,
                 y=(3. - (start + i * step)) / .01 - .5, z=0.) for i in range(count)]
    return NS(header=NS(stamp=NS(sec=stamp // 1_000_000_000, nanosec=stamp % 1_000_000_000),
                        frame_id='front_axle_bev'), source_width=120, source_height=300,
              padding_left=padding, padding_right=padding, centerline_bev_width_m=1.2,
              centerline_bev_height_m=3., state=3, centerline_valid=valid,
              centerline_sample_limit_reached=False, centerline_points=points,
              centerline_support=[2] * count, stop_line_present=True, stop_line_distance_m=.8,
              detector_sequence=1)
