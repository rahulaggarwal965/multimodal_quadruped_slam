import rospy
from cv_bridge import CvBridge
# import pykdtree

from sensor_msgs.msg import Image

from LoopClosureDL import LoopClosureDL

import numpy as np
import torch

class LoopClosure:

    def __init__(self) -> None:

        self.rgb_sub = rospy.Subscriber("/camera/color", Image, callback=self.handle_rgb)
        self.depth_sub = rospy.Subscriber("/camera/depth", Image, callback=self.handle_depth)

        #self.feature_vector_pub = rospy.Publisher("/camera/feature_vector", FeatureVector, )

        self.model = LoopClosureDL()
        self.bridge = CvBridge()

        # TODO(rahul): find a better way of synchronizing these topics
        self.rgb_frames = 0
        self.depth_frames = 0

        self.rgb_stamp = rospy.Time.now();
        self.depth_stamp = rospy.Time.now();

    def handle_rgb(self, image : Image):
        self.rgb = np.asarray(self.bridge.imgmsg_to_cv2(image), dtype=np.uint8)
        self.rgb_stamp = image.header.stamp
        self.rgb_frames += 1
        if self.rgb_frames == self.depth_frames:
            self.handle_rgbd()
        pass


    def handle_depth(self, depth : Image):
        self.depth = np.asarray(self.bridge.imgmsg_to_cv2(depth), dtype=np.uint8)
        self.depth_stamp = depth.header.stamp
        self.depth_frames += 1
        if self.rgb_frames == self.depth_frames:
            self.handle_rgbd()

    def handle_rgbd(self):
        # size of 2048
        # NOTE(rahul): because we are doing these forward passes incrementally, we can't actually
        # think about using PCA to reduce a space of feature vectors on to a subset of the eigenbasis
        # of the covariance. However, matching in a 2048 feature dim space is probably quite inefficient
        # for long horizon trajectories (especially in a robot operating in the real world.)
        # Need to think about how we can solve this problem, especially because kdtrees suffer greatly from 
        # high dimensional spaces. Perhaps we can use an ANN (see: opencv FLANN) approximate nearest neighbors
        # or we could train an encoder/decoder to reduce the dimensionality of the feature vectors themselves.
        # much work needs to be done to think about this. I think, for now, we can probably get a way with using
        # a kdtree
        # ALSO take a look at ResNetVAE https://github.com/hsinyilin19/ResNetVAE
        f = self.model.compute_hook_activation(torch.from_numpy(self.rgb).unsqueeze(0)).squeeze();
        # self.feature_vector_pub.publish(f)





