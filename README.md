# A Reduced Model for Interactive Hairs
Menglei Chai, Changxi Zheng, Kun Zhou (SIGGRAPH 2014)

## 概要
実時間で大量の髪の毛を表現する手法．頭部の様々な動きを入力し，すべての髪の毛をフルシミュレーションした結果を用いてガイドヘアを抽出し，残りのヘアをスキニングによって補間．スキニングウェイトはフルシミュレーションした結果からガイドヘアから最適なウェイトを最適化によって求める．

- 論文
	- http://gaps-zju.org/reducedhair/resources/ReducedHair.pdf
- Youtube
	- https://youtu.be/cN2phdi1AB0

## TBD
2016.03.31; スキニングウェイトの最適化部分がうまくいっていない．原因は以下．
> あるnormal particleに着目したとき，そのparticleが含まれる領域(グループ)とその領域に隣接する領域の中に含まれるguide particlesからのスキニングウェイトを求めるが，その候補が多すぎる点，また影響の遠いparticleが候補に含まれ，同時にそのparticleのウェイトが高くなってしまうことがあり，髪の毛がぐしゃぐしゃになったようなバグが発生する．

## 環境
- openFrameworks v0.8.4
	- http://www.openframeworks.cc/versions/v0.8.4/of_v0.8.4_vs_release.zip
- Visual Studio 2012
- Windows 7

## 頭髪モデル
- 以下，プロジェクトページよりダウンロード
	- http://gaps-zju.org/reducedhair/

## 使用ライブラリ
- Eigen
	- http://eigen.tuxfamily.org/index.php?title=Main_Page
- QuadProg
	- http://quadprog.sourceforge.net/
- boost 1.6.0
	- http://www.boost.org/

## 使用アドオン
以下のアドオンはaddonフォルダにあります．

- ofxNearestNeighbor
	- https://github.com/neilmendoza/ofxNearestNeighbour
- ofxUI
	- https://github.com/rezaali/ofxUI
- ofxXmlSetting
	- default

## 自作アドオン
以下のアドオンはaddonフォルダにあります．
- ofxHEMesh
- ofxRay
- ofxMeshUtil

## 使用に関する注意
- addonフォルダの中身を以下の階層に追加(of_v0.8.4_vs_relase>addons)




