import logging

from PIL import Image
import torch
from torchvision import models as M
from torchvision.io import read_video
from torchvision.transforms import transforms as T
from torchvision.transforms import functional as fn


logging.basicConfig()
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class LoopClosureDL:

    def __init__(self):
        # Retrieve a good model
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
        # TODO: Find a good model
        self.model = M.resnet18(weights=M.ResNet18_Weights.IMAGENET1K_V1).to(device)
        self.model.eval()

        # For retrieving activation outputs of intermediate layer
        self.activation = dict()
        def get_activation(component: str):
            def forward_hook(model: M, inp, out):
                # Do I need to clone here?
                # We can have multiple images, so we should flatten all dimensions except 0.
                self.activation[component] = out.detach().clone().reshape((out.shape[0], -1))
            return forward_hook

        # Reference to hook if needed for later removal/modification.
        # TODO: Find a better layer
        self.hook = self.model.layer3[0].downsample[1].register_forward_hook(get_activation("bn2"))
        self.hook = self.model.avgpool.register_forward_hook(get_activation("bn2"))

        # Matrix for SVD
        # We use vector_length as cutoff used in SVD
        # TODO: Change constant as needed depending on layer
        self.vector_length = 15
        self.matrix = None

        # A post-processing matrix reference, also added in case of future use.
        self.reduced = None
    
    def compute_hook_activation(self, input: torch.Tensor) -> torch.Tensor:
        # Later on, we can possibly modify this to only run to the intended
        # intermediate layer (if we don't need classification outputs).
        _ = self.model(input)
        if self.matrix is None:
            self.matrix = torch.Tensor().reshape((0, self.activation['bn2'].shape[1]))
        self.matrix = torch.cat((self.matrix, self.activation['bn2']), dim=0)

        print(self.matrix.shape)
        print(self.matrix)
    
    def post_processing(self):
        if self.matrix.shape[0] < 2:
            logger.info("Matrix n < 2, so no decomposition done")
            return
        
        # This section has no optimization yet.
        # This is for the purposes of creating a minimum model.

        # Zero center matrix
        X = self.matrix - torch.mean(self.matrix, dim=0)
        # Compute covariance
        cov = X.T @ X
        # Perform decomposition
        U, S, _ = torch.linalg.svd(cov)

        # Obtain reduced vector, from d to self.vector_length dimensionality.
        reduced = X @ U[:, :self.vector_length]

        # Whitening
        reduced /= torch.sqrt(S[:self.vector_length] + 1E-1)

        # Store whitened matrix here
        self.reduced = reduced

        print(self.reduced)
    
    def produce_similarity(self):
        if self.reduced is None:
            logger.info("No vectors to produce similarities for.")
            return
        # Normalize our vectors
        normal = self.reduced / torch.sqrt(torch.sum(self.reduced ** 2, axis=1)).reshape((-1, 1))

        # Compute all pair-wise dot products
        relu = torch.nn.ReLU()
        return relu(normal @ normal.T)

class DeeperLoopClosureDL(LoopClosureDL):

    def __init__(self):
        # Retrieve a good model
        device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')

        # BatchNorm2d-76          [-1, 512, 28, 28]           1,024
        self.model = M.resnet50(weights=M.ResNet50_Weights.IMAGENET1K_V2).to(device)
        self.model.eval()

        # For retrieving activation outputs of intermediate layer
        self.activation = dict()
        def get_activation(component: str):
            def forward_hook(model: M, inp, out):
                # Do I need to clone here?
                # We can have multiple images, so we should flatten all dimensions except 0.
                self.activation[component] = out.detach().clone().reshape((out.shape[0], -1))
            return forward_hook

        # Reference to hook if needed for later removal/modification.
        # TODO: Find a better layer
        self.hook = self.model.layer3[0].downsample[1].register_forward_hook(get_activation("bn2"))
        self.hook = self.model.avgpool.register_forward_hook(get_activation("bn2"))

        # Matrix for SVD
        # We use vector_length as cutoff used in SVD
        # TODO: Change constant as needed depending on layer
        self.vector_length = 15
        self.matrix = None

        # A post-processing matrix reference, also added in case of future use.
        self.reduced = None