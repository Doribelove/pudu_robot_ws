from glob import glob
import os

from setuptools import Extension, find_packages, setup

package_name = 'arena_evaluation'
package_root = os.path.dirname(os.path.realpath(__file__))

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    ext_modules=[
        Extension(
            'arena_evaluation._ompl_planner_backend',
            sources=[os.path.join(package_root, 'src', 'ompl_planner_backend.cpp')],
            include_dirs=['/usr/include/ompl-1.5', '/usr/include/eigen3'],
            libraries=['ompl', 'yaml-cpp'],
            language='c++',
            extra_compile_args=['-std=c++17', '-O3', '-Wall', '-Wextra', '-Wpedantic'],
        ),
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],                                                                          
    install_requires=['setuptools', 'numpy', 'pandas', 'pyyaml', 'scikit-image', 'matplotlib', 'Pillow'],
    zip_safe=True,
    maintainer='NamTruongTran',
    maintainer_email='trannamtruong98@gmail.com',
    description='Record, evaluate, and plot navigational metrics to evaluate ROS navigation planners',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'record = arena_evaluation.data_recorder_node:main',
        'metrics = arena_evaluation.get_metrics:main',
        'planner_benchmark = arena_evaluation.planner_benchmark.cli:main',
        'planner_benchmark_report = arena_evaluation.planner_benchmark.report:main',
        'planner_benchmark_cross_report = arena_evaluation.planner_benchmark.cross_report:main',
        'topology_benchmark = arena_evaluation.topology_cli:main',
        'kinematic_benchmark = arena_evaluation.kinematic_cli:main',
        'stage7_kinematic_benchmark = arena_evaluation.kinematic_cli:main',
        'stage8_hard_radius_benchmark = arena_evaluation.stage8_cli:main',
        'stage8a_hard_radius_l3 = arena_evaluation.stage8_cli:main',
        'stage8_lateral_preference = arena_evaluation.preference_cli:main',
        'stage8b_lateral_preference = arena_evaluation.preference_cli:main',
        'stage8_report = arena_evaluation.stage8_report:main',
        'prepare_fixed_resolution_map = arena_evaluation.fixed_resolution_map:main',
        'prepare_padded_map = arena_evaluation.padded_map:main',
        'stage5_resolution_report = arena_evaluation.stage5_report:main',
        'planner_resolution_comparison = arena_evaluation.resolution_comparison:main',
        'scale_benchmark = arena_evaluation.scale_benchmark:main',
        'single_planner_benchmark = arena_evaluation.single_planner_benchmark:main',
        'relaxed_single_planner_benchmark = arena_evaluation.relaxed_single_planner_benchmark:main',
        'forward_no_reverse_smoke = arena_evaluation.forward_no_reverse_smoke:main',
        'unified_four_backends_smoke = arena_evaluation.unified_four_backends_smoke:main',
        'fixed_layered_pipeline_smoke = arena_evaluation.fixed_layered_pipeline_smoke:main',
        'fixed_layered_pipeline_efficiency_smoke = arena_evaluation.fixed_layered_pipeline_efficiency_smoke:main',
        'l1_l3_corridor_hybrid_smoke = arena_evaluation.l1_l3_corridor_hybrid_smoke:main',
        'layered_architecture_paired_benchmark = arena_evaluation.layered_architecture_paired_benchmark:main',
        'forward_no_reverse_repair_smoke = arena_evaluation.forward_no_reverse_repair_smoke:main',
        'single_planner_report = arena_evaluation.single_planner_report:main',
        'baseline_path_visualizer = arena_evaluation.baseline_visualizer:main',
        'layered_path_visualizer = arena_evaluation.layered_visualizer:main',
        'layered_pipeline_visualize = arena_evaluation.layered_pipeline_visualize:main',
        ],
    },
)
