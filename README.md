# LogGPisMap 

This repository contains source code for LogGPisMap, introduced in the paper [Faithful Euclidean distance field from Log-Gaussian Process Implicit Surfaces](https://ieeexplore.ieee.org/abstract/document/9361242) presented at RA-L and ICRA2021. 

The source code is modified from Bhoaram Lee's [GPisMap code](https://github.com/leebhoram/GPisMap).

## License

Licensed under [GNU General Public License version 3](https://www.gnu.org/licenses/gpl-3.0.html).

## Requirements: Software

1. [Eigen](http://eigen.tuxfamily.org/)

2. [MATLAB](https://www.mathworks.com/products/matlab.html)

## Compiling and Running

1. Clone this repository
```
git clone https://github.com/lanwu076/LogGPisMap.git
```

2. Go into the mex directory in MATLAB
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
    ```

4. Run the demo scripts

    * For 2D 
    ```
    run('../matlab/demo_gpisMap.m')
    ```

5. Trouble shooting
    * If mex complains about not finding eigen, configure the eigen path appropriately
        in `make_GPisMap.m`

## Video  

A demonstration video is available [here](https://www.youtube.com/watch?v=bSGx_WNdQvo).

## Contributors

The major contributors of this work are [Lan Wu](https://github.com/lanwu076), [Ki Myung Brian Lee](https://github.com/lkm1321).

## Citation
 
If you find LogGPisMap useful in your research, please consider citing:
```
@ARTICLE{lwu21-ral,
  author={Wu, Lan and Lee, Ki Myung Brian and Liu, Liyang and Vidal-Calleja, Teresa},
  journal={Rob. and Automat. Lett.}, 
  title={Faithful Euclidean Distance Field From Log-Gaussian Process Implicit Surfaces}, 
  year={2021},
  volume={6},
  number={2},
  pages={2461-2468}
}
```
