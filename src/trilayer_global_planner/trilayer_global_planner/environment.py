# SPDX-License-Identifier: Apache-2.0
"""Refuse transport overrides that can undermine this entry's localhost contract."""
# Do not load/read the referenced files, keys, profiles or discovery endpoints.
UNSUPPORTED_OVERRIDES=(
    'FASTRTPS_DEFAULT_PROFILES_FILE','FASTDDS_DEFAULT_PROFILES_FILE','CYCLONEDDS_URI',
    'ROS_DISCOVERY_SERVER','ROS_STATIC_PEERS','ROS_AUTOMATIC_DISCOVERY_RANGE',
    'RMW_FASTRTPS_USE_QOS_FROM_XML','ROS_SECURITY_ENABLE','ROS_SECURITY_STRATEGY',
    'ROS_SECURITY_KEYSTORE','ROS_SECURITY_ENCLAVE_OVERRIDE','ROS_NAMESPACE',
    'LD_PRELOAD','PYTHONHOME',
)


def check_environment(env):
    active=[name for name in UNSUPPORTED_OVERRIDES if env.get(name)]
    if active:
        raise ValueError('Unsupported environment overrides for isolated planning: '+', '.join(active)+'. Use a clean shell with only the installed ROS/package setup.')
