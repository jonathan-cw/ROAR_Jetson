#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sun Jun 25 10:44:24 2017

@author: wroscoe
"""

import time
from threading import Thread
from ROAR_Jetson.arduino_cmd_sender import ArduinoCommandSender
import logging
from ROAR_Jetson.camera import RS_D435i
from typing import Optional, List
import numpy as np
from ROAR_Jetson.arduino_receiver import ArduinoReceiver
from ROAR_Jetson.vive.vive_tracker_client import ViveTrackerClient


class Vehicle:
    """
    In right hand coordinate system
    """

    def __init__(self):
        self.parts = []
        self.on = True
        self.threads = []
        self.logger = logging.getLogger("Jetson Vehicle")
        self.front_rgb_img: Optional[np.ndarray] = None
        self.front_depth_img: Optional[np.ndarray] = None
        self.location: np.array = np.asarray([0, 0, 0])  # x,y,z
        self.rotation: np.array = np.asarray([0, 0, 0])  # pitch, yaw, roll
        self.velocity: np.array = np.asarray([0, 0, 0])  # vel_x, vel_y, vel_z
        self.throttle: float = 0
        self.steering: float = 0

    def add(self, part, inputs: Optional[List] = None, outputs: Optional[List] = None,
            threaded=False):
        """
        Method to add a part to the vehicle drive loop.

        Args:
            part: Generic object that has a run_threaded and update method
            inputs: input to the parts
            outputs: Channel names to save to memory.
            threaded: If a part should be run in a separate thread.

        Returns:
            None
        """
        if outputs is None:
            outputs = []
        if inputs is None:
            inputs = []
        assert type(inputs) is list, "inputs is not a list: %r" % inputs
        assert type(outputs) is list, "outputs is not a list: %r" % outputs
        assert type(threaded) is bool, "threaded is not a boolean: %r" % threaded

        p = part
        self.logger.info('Adding part {}.'.format(p.__class__.__name__))
        entry = dict()
        entry['part'] = p
        entry['inputs'] = inputs
        entry['outputs'] = outputs

        if threaded:
            t = Thread(target=part.update, args=())
            t.daemon = True
            entry['thread'] = t

        self.parts.append(entry)
        self.logger.debug(f"{p.__class__.__name__} successfully added")

    def remove(self, part):
        """
        remove part form list
        """
        self.parts.remove(part)

    def start(self, rate_hz=10):
        """
        Start vehicle's main drive loop.

        This is the main thread of the vehicle. It starts all the new
        threads for the threaded parts then starts an infinite loop
        that runs each part.

        Notes: This function is NOT used during integration with ROAR_Sim

        Args:
            rate_hz : int, The max frequency that the drive loop should run. The actual
            frequency may be less than this if there are many blocking parts.
        """

        try:
            for entry in self.parts:
                if entry.get('thread'):
                    # start the update thread
                    entry.get('thread').start()

            # wait until the parts warm up.
            self.logger.info('Starting vehicle...')

            while True:
                start_time = time.time()

                self.update_parts()

                sleep_time = 1.0 / rate_hz - (time.time() - start_time)
                if sleep_time > 0.0:
                    time.sleep(sleep_time)
                else:
                    pass
        except KeyboardInterrupt:
            self.stop()
        except Exception as e:
            self.logger.error(f"Something bad happened: [{e}]")
        finally:
            self.stop()

    def start_part_threads(self):
        for entry in self.parts:
            if entry.get('thread'):
                # start the update thread
                p = entry.get('part')
                entry.get('thread').start()
                self.logger.debug(f"{p.__class__.__name__} thread started")

    def update_parts(self, new_throttle: float = 0, new_steering: float = 0):
        """
        Fail-safe method for loop over all parts and call update on each one
        Notes: This function IS USED during integration with ROAR Sim

        Args:
            new_throttle:  new throttle value, gaurenteed between -1 and 1
            new_steering:  new steering value, gaurenteed between -1 and 1

        Returns:
            None
        """
        for entry in self.parts:
            try:
                p = entry["part"]
                if entry.get('thread') and isinstance(p, ArduinoCommandSender):
                    # send the throttle and steering to Arduino
                    p.run_threaded(throttle=new_throttle, steering=new_steering)
                    self.throttle, self.steering = new_throttle, new_steering
                elif entry.get('thread') and isinstance(p, RS_D435i):
                    self.front_rgb_img, self.front_depth_img = p.run_threaded()
                elif isinstance(p, ArduinoReceiver):
                    pass
                elif isinstance(p, ViveTrackerClient):
                    p.run_threaded()
                else:
                    self.logger.warning(f"Unknown part [{p}]")
            except KeyboardInterrupt as e:
                exit(0)  # this is a hack for existing the program. DON"T CHANGE!
            except Exception as e:
                self.logger.error(f"Something bad happened during run_threaded for part [{entry}]: {e}")

    def stop(self):
        """
        Stop all parts attached to the Jetson vehicle

        Returns:
            None
        """
        self.logger.info('Shutting down vehicle and its parts...')
        for entry in self.parts:
            try:
                entry['part'].shutdown()
            except AttributeError:
                # usually from missing shutdown method, which should be optional
                pass
            except Exception as e:
                self.logger.error(e)
