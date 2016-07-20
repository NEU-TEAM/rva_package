#include "vgg_process.h"


void vgg_process::bubble_sort(float *feature, int *sorted_idx)
{
		int i=0, j=0;
		float tmp;
		int tmp_idx;

		for(i=0; i<1000; i++)
				sorted_idx[i] = i;

		for(i=0; i<1000; i++)
				{
						for(j=0; j<999; j++)
								{
										if(feature[j] < feature[j+1])
												{
														tmp = feature[j];
														feature[j] = feature[j+1];
														feature[j+1] = tmp;

														tmp_idx = sorted_idx[j];
														sorted_idx[j] = sorted_idx[j+1];
														sorted_idx[j+1] = tmp_idx;
												}
								}
				}
}

void vgg_process::get_top5(float *feature, int arr[5])
{
		int i=0;
		int sorted_idx[1000];

		bubble_sort(feature, sorted_idx);

		for(i=0; i<5; i++)
				{
						arr[i] = sorted_idx[i];
				}
}

void vgg_process::get_label(char filename[256], char label[][512])
{
		FILE *fp = fopen(filename, "r");

		int i=0, j=0;


		for(i=0; i<1000; i++)
				{
						fgets(label[i], 512, fp);

						for(j=0; j<512; j++)
								{
										if(label[i][j] == '\n')
												label[i][j] = '\0';
								}
				}

		fclose(fp);
}

const string& deploy = "/home/aicrobo/deeplearning/VGG_ILSVRC_19_layers_deploy.prototxt";
Net<float> caffe_test_net(deploy, TEST);

vgg_process::vgg_process()
{
		get_label("/home/aicrobo/deeplearning/data/synset_words.txt", label);

		// mode setting - CPU/GPU
		Caffe::set_mode(Caffe::GPU);

		// gpu device number
		int device_id = 0;
		Caffe::SetDevice(device_id);

		caffe_test_net.CopyTrainedLayersFrom("/home/aicrobo/deeplearning/VGG_ILSVRC_19_layers.caffemodel");
}


bool vgg_process::process(cv::Mat image_in, string &name, float &conf)
{
    int i=0, j=0, k=0;
    int top5_idx[5];
    float mean_val[3] = {103.939, 116.779, 123.68}; // bgr mean

    // input
    float output[1000];
    vector<Blob<float>*> input_vec;
    Blob<float> blob(1, 3, IMAGE_SIZE, IMAGE_SIZE);

    cv::Mat image_small(cv::Size(IMAGE_SIZE, IMAGE_SIZE), CV_8UC3);

    cv::resize(image_in, image_small, cv::Size(IMAGE_SIZE, IMAGE_SIZE));

    for (k=0; k<3; k++)
        {
            for (i=0; i<IMAGE_SIZE; i++)
                {
                    for (j=0; j< IMAGE_SIZE; j++)
                        {
                            blob.mutable_cpu_data()[blob.offset(0, k, i, j)] = (float)(unsigned char)image_small.data[i*image_small.step+j*image_small.channels()+k] - mean_val[k];
                        }
                }
        }

    input_vec.push_back(&blob);

    // forward propagation
    float loss;

    const vector<Blob<float>*>& result = caffe_test_net.Forward(input_vec, &loss);

    // copy output
    for(i=0; i<1000; i++)
        {
            output[i] = result[0]->cpu_data()[i];
        }

    get_top5(output, top5_idx);

    char str[256];
    sprintf(str, "%s", label[top5_idx[0]]);
    name = str;

    conf = output[0];

    if (conf < 0.3)
        return false;
    else
        return true;
}
