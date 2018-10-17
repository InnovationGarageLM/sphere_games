import simpleagent

b = simpleagent.SimpleAgent('red_sphero', opponent='blue_sphero')

b.setup_ros()

b.play_game()
