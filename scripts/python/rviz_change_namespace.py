import os
import yaml
import argparse


parser = argparse.ArgumentParser(description='Change RViz Namespace Script Arguments')
parser.add_argument(
    '-c', '--config_file_path', 
    help='absolute path of rviz2 configuration file path')
parser.add_argument(
    '-n', '--namespace', 
    help='namespace',
    default='')
args = parser.parse_args()


def modify_rviz_config_file():
    namespace = args.namespace
    if len(namespace) > 0:
        if namespace[0] != '/':
            namespace = '/' + namespace

    if args.config_file_path is None or not os.path.isfile(args.config_file_path):
        print("Invalid rviz config file path!")
        return
    
    with open(args.config_file_path, 'r') as fp:
        rviz_config = yaml.load(fp, Loader=yaml.FullLoader)
    
    display_list = rviz_config['Visualization Manager']['Displays']
    find = list(filter(lambda x: x.get('Class') == 'rviz_default_plugins/RobotModel', display_list))
    for elem in find:
        elem['Description Topic']['Value'] = namespace + '/robot_description'
    find = list(filter(lambda x: x.get('Class') == 'moveit_rviz_plugin/MotionPlanning', display_list))
    for elem in find:
        elem['Move Group Namespace'] = namespace
    
    with open(args.config_file_path, 'w') as fp:
        yaml.dump(rviz_config, fp)


def main():
    modify_rviz_config_file()


if __name__ == '__main__':
    main()
