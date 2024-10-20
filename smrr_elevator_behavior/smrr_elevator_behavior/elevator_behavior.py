from time import sleep
from py_trees.behaviour import Behaviour
from py_trees.common import Status
from py_trees.composites import Sequence
from py_trees import logging as log_tree


class LineEstimation(Behaviour):
  def __init__(self, name):
    super(LineEstimation, self).__init__(name)

  def setup(self):
    self.logger.debug(f"LineEstimation::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"LineEstimation::initialise {self.name}")

  def update(self):
    self.logger.debug(f"LineEstimation::update {self.name}")
    sleep(1)
    return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"LineEstimation::terminate {self.name} to {new_status}")


class ButtonEstimation(Behaviour):
  def __init__(self, name):
    super(ButtonEstimation, self).__init__(name)

  def setup(self):
    self.logger.debug(f"ButtonEstimation::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"ButtonEstimation::initialise {self.name}")

  def update(self):
    self.logger.debug(f"ButtonEstimation::update {self.name}")
    sleep(1)
    return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"ButtonEstimation::terminate {self.name} to {new_status}")


class ButtonLocalization(Behaviour):
  def __init__(self, name):
    super(ButtonLocalization, self).__init__(name)

  def setup(self):
    self.logger.debug(f"ButtonLocalization::setup {self.name}")

  def initialise(self):
    self.logger.debug(f"ButtonLocalization::initialise {self.name}")

  def update(self):
    self.logger.debug(f"ButtonLocalization::update {self.name}")
    sleep(1)
    return Status.SUCCESS

  def terminate(self, new_status):
    self.logger.debug(f"ButtonLocalization::terminate {self.name} to {new_status}")


def make_bt():
  root = Sequence(name="elevator_interaction", memory=True)

  button_estimation     = ButtonEstimation("estimate_button")
  line_estimation       = LineEstimation("estimate_line")
  button_localization   = LineEstimation("button_localization")
 

  root.add_children(
      [
          button_estimation,
          line_estimation,
          button_localization
      ]
  )

  return root


if __name__ == "__main__":
  log_tree.level = log_tree.Level.DEBUG
  tree = make_bt()
  tree.tick_once()