# Motion Capture Interpolation

Implementation of 3 methods of interpolation :
* [Bezier interpolation for Euler angles.](#1-input--bezier-euler)
* [SLERP interpolation for quaternions.](#2-input--slerp-quaternion)
* [Bezier SLERP interpolation for quaternions.](#3-input--bezier-slerp-quaternion)

& comparison of the methods to determine their effectiveness and their drawbacks.\
<br/>

## Plotted Graph Comparisons

![Linear vs Bezier Euler Graph](/results/graphs/Linear%20Euler%20+%20Bezier%20Euler%20interpolation%20(+%20Input)%20-%20Graph%201.png)

![SLERP vs Bezier SLERP Quaternion Graph](/results/graphs/SLERP%20Quaternion%20+%20Bezier%20SLERP%20Quaternion%20Interpolation%20(+%20Input)%20-%20Graph%202.png)

![Linear Euler vs SLERP Quaternion Graph](/results/graphs/Linear%20Euler%20+%20SLERP%20Quaternion%20(+%20Input)%20-%20Graph%203.png)

![Bezier Euler vs Bezier SLERP Quaternion Graph](/results/graphs/Bezier%20Euler%20+%20Bezier%20SLERP%20Quaternion%20(+%20Input)%20-%20Graph%204.png)


## Analysis + Observations

On testing and comparing the methods :
* Bezier Euler is visibly better than Linear Euler. However, there is still a sharp abrupt motion which is quite undesirable.
* The Linear method has very sharp curves, indicating dramatic motion change in some frames.
* The Linear Quaternion method works quite well too, and is quite close to the input motion, however, sharp motion changes still exist.

<br/>

**The Bezier Quaternion method provides a much better result and avoids this problem.**

**Although it is computationally more expensive, the smoothness of the motion is nearly identical to the input in most cases, and makes it worth the cost.**

**This is evident in the graphs above, as the curves are smoother.**\
<br/>

## Comparison of Computation Time

I compared the time values of the 4 methods on the same skeleton and motion file. The computational time required by the methods is as follows in increasing order (where N=10), i.e.

**_Linear for Euler (0.065) < SLERP for quaternions (0.399) < Bezier for Euler (0.542) < Bezier SLERP for quaternions (1.198)_**

So Linear interpolation for Euler takes the least time, and Bezier SLERP interpolation for quaternions takes the most time, but has a much greater effectiveness.\
<br/>

## Conclusion

Linear interpolation may be quick, but the result is very sharp and undesirable.\
Bezier quaternion interpolation on the other hand delivers a much better angular velocity that is smoother and preferred, and is worth its computational cost.\
It is understandable how quaternions have become a standard in most industries.\
<br/>

## Comparison GIFs

### 1. Input + Bezier Euler
![1. Input + Bezier Euler GIF](/results/1.gif)

### 2. Input + SLERP Quaternion
![2. Input + SLERP quaternion GIF](/results/2.gif)

### 3. Input + Bezier SLERP Quaternion
![3. Input + Bezier SLERP quaternion GIF](/results/3.gif)


