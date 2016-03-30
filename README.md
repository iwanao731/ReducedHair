# A Reduced Model for Interactive Hairs

## 概要
実時間で大量の髪の毛を表現する手法．頭部の様々な動きを入力し，すべての髪の毛をフルシミュレーションした結果を用いてガイドヘアを抽出し，残りのヘアをスキニングによって補間．スキニングウェイトはフルシミュレーションした結果からガイドヘアから最適なウェイトを最適化によって求める．

## TBD
2016.03.31; スキニングウェイトの最適化部分がうまくいっていない．原因は以下．
> あるnormal particleに着目したとき，そのparticleが含まれる領域(グループ)とその領域に隣接する領域の中に含まれるguide particlesからのスキニングウェイトを求めるが，その候補が多すぎる点，また影響の遠いparticleが候補に含まれ，同時にそのparticleのウェイトが高くなってしまうことがあり，髪の毛がぐしゃぐしゃになったようなバグが発生する．

## 環境
- openFrameworks v0.8.4
- Visual Studio 2012
- Windows 7

## 使用ライブラリ
- Eigen
	- http://eigen.tuxfamily.org/index.php?title=Main_Page
- QuadProg
	- http://quadprog.sourceforge.net/
- boost 1.6.0
	- http://www.boost.org/

## 使用アドオン
- ofxNearestNeighbor
	- https://github.com/neilmendoza/ofxNearestNeighbour
- ofxUI
	- https://github.com/rezaali/ofxUI
- ofxXmlSetting
	- default

## 自作アドオン
- ofxHEMesh
- ofxRay
- ofxMeshUtil



