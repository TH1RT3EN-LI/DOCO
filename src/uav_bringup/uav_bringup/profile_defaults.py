VEHICLE_PROFILES = {
    "sim": {
        "frame": "sim_uav",
        "fx": "387.229248046875",
        "fy": "387.229248046875",
        "cx": "321.04638671875",
        "cy": "243.44969177246094",
        "cam2body_x": "0.08",
        "cam2body_y": "0.0",
        "cam2body_z": "0.03",
    },
    "hw": {
        "frame": "hw_uav",
        "fx": "387.229248046875",
        "fy": "387.229248046875",
        "cx": "321.04638671875",
        "cy": "243.44969177246094",
        "cam2body_x": "0.08",
        "cam2body_y": "0.0",
        "cam2body_z": "0.03",
    },
}


def _quote(value):
    return '"' + str(value) + '"'


def profile_expression(profile_config, selector, key):
    expression = []
    items = list(profile_config.items())

    for index, (name, values) in enumerate(items):
        if index == 0:
            expression.extend([
                _quote(values[key]),
                ' if "',
                selector,
                '" == "',
                name,
                '"',
            ])
        else:
            expression.extend([
                ' else ',
                _quote(values[key]),
                ' if "',
                selector,
                '" == "',
                name,
                '"',
            ])

    expression.extend([
        ' else ',
        _quote(items[0][1][key]),
    ])
    return expression
