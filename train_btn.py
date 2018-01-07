#!/usr/bin/env python 
from __future__ import division

import os
import tensorflow as tf
import model_btn as model
import params_btn as params
import time

if params.shuffle_training:
    import data_shuffled_btn as data
else:
    import data_ordered as data


import local_common as cm

write_summary = params.write_summary

sess = tf.InteractiveSession()
loss = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=model.y_lg, labels=model.y_oh_))
# loss = tf.reduce_mean(tf.square(tf.subtract(model.y_, model.y)))
# loss = tf.reduce_mean(tf.square(tf.sub(model.y_, model.y)))
#         + tf.add_n([tf.nn.l2_loss(v) for v in train_vars]) * L2NormConst
train_step = tf.train.AdamOptimizer(1e-4).minimize(loss)

sess.run(tf.global_variables_initializer())


# create a summary to monitor cost tensor
if write_summary:
    tf.summary.scalar("loss", loss)

# merge all summaries into a single op
if write_summary:
    merged_summary_op = tf.summary.merge_all()

saver = tf.train.Saver()
time_start = time.time()

# op to write logs to Tensorboard
if write_summary:
    summary_writer = tf.summary.FileWriter(params.save_dir, graph=tf.get_default_graph())

if params.shuffle_training:
    data.load_imgs()

    # norm_input = sess.run(model.yy_, feed_dict={model.x: txx, model.y_: tyy, model.keep_prob: 1.0})    
    # enco_input = sess.run(model.y_oh_yy_, feed_dict={model.x: txx, model.y_: tyy, model.keep_prob: 1.0})
    # encoded_input = sess.run(model.y_oh_, feed_dict={model.x: txx, model.y_: tyy, model.keep_prob: 1.0})    
    # for n, e, p,y in zip(norm_input, enco_input, encoded_input, tyy):
    #     print("N={}, E={}, E2={}, O={}".format(n, e, p, y))
    
for i in xrange(params.training_steps):
    txx, tyy = data.load_batch('train')
              
    train_step.run(feed_dict={model.x: txx, model.y_: tyy, model.keep_prob: 0.8})

    # write logs at every iteration
    if write_summary:
        summary = merged_summary_op.eval(feed_dict={model.x: txx, model.y_: tyy, model.keep_prob: 1.0})
        summary_writer.add_summary(summary, i)

    if (i+1) % 10 == 0 or i < 10:
        vxx, vyy = data.load_batch('val')
        t_loss = loss.eval(feed_dict={model.x: txx, model.y_: tyy, model.keep_prob: 1.0})
        v_loss = loss.eval(feed_dict={model.x: vxx, model.y_: vyy, model.keep_prob: 1.0})
        print "step {} of {}, train loss {}, val loss {}".format(i+1, params.training_steps, t_loss, v_loss)

    if (i+1) % 100 == 0:
        if not os.path.exists(params.save_dir):
            os.makedirs(params.save_dir)
        checkpoint_path = os.path.join(params.save_dir, model_name)
        filename = saver.save(sess, checkpoint_path)

        time_passed = cm.pretty_running_time(time_start)
        time_left = cm.pretty_time_left(time_start, i, params.training_steps)
        print 'Model saved. Time passed: {}. Time left: {}'.format(time_passed, time_left) 
        
# Let's see if we can predict
vxx, vyy = data.load_batch('val')

prediction = tf.argmax(model.y_lg, 1) 
correct_prediction = tf.equal(prediction, tf.argmax(model.y_oh_, 1))
accuracy = tf.reduce_mean(tf.cast(correct_prediction, tf.float32))

print ('Accuracy:',
       sess.run(accuracy, feed_dict={model.x: vxx, model.y_: vyy, model.keep_prob: 1.0}))

# print ('Labels:',
#        sess.run(model.y_oh_, feed_dict={model.x: vxx, model.y_: vyy, model.keep_prob: 1.0}))
print ("Prediction: ",
       sess.run(prediction, feed_dict={model.x: vxx, model.y_: vyy, model.keep_prob: 1.0}))

# y_data: (N,1) = flatten => (N, ) matches pred.shape
# for p, y in zip(pred, model.y_lg.flatten()):
#     print("[{}] Prediction: {} True Y: {}".format(p == int(y), p, int(y)))
