from agents.simpleagent import SimpleAgent
import host.constants as constants

red_sphero = SimpleAgent(name='red_sphero', opponent='blue_sphero')
blue_sphero = SimpleAgent(name='red_sphero', opponent='blue_sphero')

red_sphero.setup_ros()
blue_sphero.setup_ros()

red_sphero.reset_position()
red_sphero.go_via_path(constants.BASE['red_sphero'])

blue_sphero.reset_position()
blue_sphero.go_via_path(constants.BASE['blue_sphero'])