# LogGPisMap 

This repository is developped based on the open source code from [link](https://github.com/leebhoram/GPisMap)
Please find our paper Faithful Euclidean Distance Field From Log-Gaussian Process Implicit Surfaces [here](https://arxiv.org/abs/2010.11487)

## Contributors

The major contributors of this work include [Brian Lee](https://github.com/lkm1321).


## License

Licensed under [GNU General Public License version 3](https://www.gnu.org/licenses/gpl-3.0.html).

## Requirements: Software

1. [Eigen](http://eigen.tuxfamily.org/)

2. [MATLAB](https://www.mathworks.com/products/matlab.html)

## Compiling and Running

1. Clone this repository
```
git clone https://github.com/leebhoram/GPisMap.git
```

2. Cd to the mex directive in MATLAB
```
cd mex
```

3. Compile the mex functions by executing the make script.
    * Setup mex 
    ```
    mex -setup
    ```
    * Run the make scripts
    ```
    make_GPisMap
    make_GPisMap3
    ```

4. Run the demo scripts

    * For 2D 
    ```
    run('../matlab/demo_gpisMap.m')
    ```
    * For 3D 
    ```
    run('../matlab/demo_gpisMap3.m')
    ```

5. Trouble shooting
    * If mex complains about not finding eigen, configure the eigen path appropriately
        in both `make_GPisMap.m` and `make_GPisMap3.m`

## Misc.

Code has been tested under:

- Ubuntu 18.04 with Intel Core i7 @ 2.50GHz

## Citation
 
Please consider citing:

```
@article{wu2021faithful,
  title={Faithful Euclidean distance field from log-Gaussian process implicit surfaces},
  author={Wu, Lan and Lee, Ki Myung Brian and Liu, Liyang and Vidal-Calleja, Teresa},
  journal={IEEE Robotics and Automation Letters},
  volume={6},
  number={2},
  pages={2461--2468},
  year={2021},
  publisher={IEEE}
}
```

```
  @article{<blee-icra19>,
      Author = {Bhoram Lee, Clark Zhang, Zonghao Huang, and Daniel D. Lee},
      Title = {Online Continuous Mapping using Gaussian Process Implicit Surfaces},
      Journal = {IEEE ICRA},
      Year = {2019}
   }    
```
