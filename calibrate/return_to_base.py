from agents.simpleagent import SimpleAgent
import host.constants as constants
import host.utilities as util

red_sphero = SimpleAgent(name='red_sphero', opponent='blue_sphero')
red_sphero.setup_ros()
red_sphero.reset_position()
red_base_mm = util.pixels_2_mm(constants.BASE['red_sphero'])
red_sphero.go_via_path(red_base_mm, red_sphero.return_false)
rospy

blue_sphero = SimpleAgent(name='blue_sphero', opponent='red_sphero')
blue_sphero.setup_ros()
blue_sphero.reset_position()
blue_base_mm = util.pixels_2_mm(constants.BASE['blue_sphero'])
blue_sphero.go_via_path(blue_base_mm, blue_sphero.return_false)