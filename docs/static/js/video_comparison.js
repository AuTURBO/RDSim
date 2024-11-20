// This is based on: http://thenewcode.com/364/Interactive-Before-and-After-Video-Comparison-in-HTML5-Canvas
// With additional modifications based on: https://jsfiddle.net/7sk5k4gp/13/

function playVids(videoId) {
    var videoMerge = document.getElementById(videoId + "Merge");
    var vid = document.getElementById(videoId);

    var position = 0.5;
    var vidWidth = vid.videoWidth/2;
    var vidHeight = vid.videoHeight;

    var mergeContext = videoMerge.getContext("2d");

    
    if (vid.readyState > 3) {
        vid.play();

        function trackLocation(e) {
            // Normalize to [0, 1]
            bcr = videoMerge.getBoundingClientRect();
            position = ((e.pageX - bcr.x) / bcr.width);
        }
        function trackLocationTouch(e) {
            // Normalize to [0, 1]
            bcr = videoMerge.getBoundingClientRect();
            position = ((e.touches[0].pageX - bcr.x) / bcr.width);
        }

        videoMerge.addEventListener("mousemove",  trackLocation, false); 
        videoMerge.addEventListener("touchstart", trackLocationTouch, false);
        videoMerge.addEventListener("touchmove",  trackLocationTouch, false);


        function drawLoop() {
            mergeContext.drawImage(vid, 0, 0, vidWidth, vidHeight, 0, 0, vidWidth, vidHeight);
            var colStart = (vidWidth * position).clamp(0.0, vidWidth);
            var colWidth = (vidWidth - (vidWidth * position)).clamp(0.0, vidWidth);
            mergeContext.drawImage(vid, colStart+vidWidth, 0, colWidth, vidHeight, colStart, 0, colWidth, vidHeight);
            requestAnimationFrame(drawLoop);

            
            var arrowLength = 0.09 * vidHeight;
            var arrowheadWidth = 0.025 * vidHeight;
            var arrowheadLength = 0.04 * vidHeight;
            var arrowPosY = vidHeight / 10;
            var arrowWidth = 0.007 * vidHeight;
            var currX = vidWidth * position;

            // Draw circle
            mergeContext.arc(currX, arrowPosY, arrowLength*0.7, 0, Math.PI * 2, false);
            mergeContext.fillStyle = "#FFD79340";
            mergeContext.fill()
            //mergeContext.strokeStyle = "#444444";
            //mergeContext.stroke()
            
            // Draw border
            mergeContext.beginPath();
            mergeContext.moveTo(vidWidth*position, 0);
            mergeContext.lineTo(vidWidth*position, vidHeight);
            mergeContext.closePath()
            mergeContext.strokeStyle = "#444444";
            mergeContext.lineWidth = 5;            
            mergeContext.stroke();

            // Draw arrow
            mergeContext.beginPath();
            mergeContext.moveTo(currX, arrowPosY - arrowWidth/2);
            
            // Move right until meeting arrow head
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowWidth/2);
            
            // Draw right arrow head
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowheadWidth/2);
            mergeContext.lineTo(currX + arrowLength/2, arrowPosY);
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowheadWidth/2);
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowWidth/2);

            // Go back to the left until meeting left arrow head
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowWidth/2);
            
            // Draw left arrow head
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowheadWidth/2);
            mergeContext.lineTo(currX - arrowLength/2, arrowPosY);
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY  - arrowheadWidth/2);
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY);
            
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY - arrowWidth/2);
            mergeContext.lineTo(currX, arrowPosY - arrowWidth/2);

            mergeContext.closePath();

            mergeContext.fillStyle = "#444444";
            mergeContext.fill();

            
            
        }
        requestAnimationFrame(drawLoop);
    } 
}

Number.prototype.clamp = function(min, max) {
  return Math.min(Math.max(this, min), max);
};
    
    
function resizeAndPlay(element)
{
  var cv = document.getElementById(element.id + "Merge");
  cv.width = element.videoWidth/2;
  cv.height = element.videoHeight;

  console.log("video height:" + element.videoHeight)
  console.log("video width:" + element.videoWidth)

  element.play();
  element.style.height = "0px";  // Hide video without stopping it
    
  playVids(element.id);
}





// This is based on: http://thenewcode.com/364/Interactive-Before-and-After-Video-Comparison-in-HTML5-Canvas
// With additional modifications based on: https://jsfiddle.net/7sk5k4gp/13/

function playVidsDual(videoId, videoId2) {
    var videoMerge = document.getElementById(videoId + "Merge");
    var vid = document.getElementById(videoId);

    var videoMerge2 = document.getElementById(videoId2 + "Merge");
    var vid2 = document.getElementById(videoId2);

    var position = 0.5;
    var vidWidth = vid.videoWidth/2;
    var vidHeight = vid.videoHeight;

    var mergeContext = videoMerge.getContext("2d");
    var mergeContext2 = videoMerge2.getContext("2d");

    
    if ((vid.readyState > 3) && (vid2.readyState > 3)) {
        vid.play();
        vid2.play();

        function trackLocation(e) {
            // Normalize to [0, 1]
            bcr = videoMerge.getBoundingClientRect();
            position = ((e.pageX - bcr.x) / bcr.width);

            if (position > 1){
                position = position-1
            }
            // console.log("pos1: " + position)
        }
        function trackLocationTouch(e) {
            // Normalize to [0, 1]
            bcr = videoMerge.getBoundingClientRect();
            position = ((e.touches[0].pageX - bcr.x) / bcr.width);

            if (position > 1){
                position = position-1
            }
        }

        videoMerge.addEventListener("mousemove",  trackLocation, false); 
        videoMerge.addEventListener("touchstart", trackLocationTouch, false);
        videoMerge.addEventListener("touchmove",  trackLocationTouch, false);
        
        videoMerge2.addEventListener("mousemove",  trackLocation, false); 
        videoMerge2.addEventListener("touchstart", trackLocationTouch, false);
        videoMerge2.addEventListener("touchmove",  trackLocationTouch, false);


        function drawLoop() {
            mergeContext.drawImage(vid, 0, 0, vidWidth, vidHeight, 0, 0, vidWidth, vidHeight);
            mergeContext2.drawImage(vid2, 0, 0, vidWidth, vidHeight, 0, 0, vidWidth, vidHeight);
            var colStart = (vidWidth * position).clamp(0.0, vidWidth);
            var colWidth = (vidWidth - (vidWidth * position)).clamp(0.0, vidWidth);
            mergeContext.drawImage(vid, colStart+vidWidth, 0, colWidth, vidHeight, colStart, 0, colWidth, vidHeight);
            mergeContext2.drawImage(vid2, colStart+vidWidth, 0, colWidth, vidHeight, colStart, 0, colWidth, vidHeight);
            requestAnimationFrame(drawLoop);

            
            var arrowLength = 0.09 * vidHeight;
            var arrowheadWidth = 0.025 * vidHeight;
            var arrowheadLength = 0.04 * vidHeight;
            var arrowPosY = vidHeight / 10;
            var arrowWidth = 0.007 * vidHeight;
            var currX = vidWidth * position;

            // Draw circle
            mergeContext.arc(currX, arrowPosY, arrowLength*0.7, 0, Math.PI * 2, false);
            mergeContext.fillStyle = "#FFD79340";
            mergeContext.fill()
            //mergeContext.strokeStyle = "#444444";
            //mergeContext.stroke()
            
            // Draw border
            mergeContext.beginPath();
            mergeContext.moveTo(vidWidth*position, 0);
            mergeContext.lineTo(vidWidth*position, vidHeight);
            mergeContext.closePath()
            mergeContext.strokeStyle = "#444444";
            mergeContext.lineWidth = 5;            
            mergeContext.stroke();

            // Draw arrow
            mergeContext.beginPath();
            mergeContext.moveTo(currX, arrowPosY - arrowWidth/2);
            
            // Move right until meeting arrow head
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowWidth/2);
            
            // Draw right arrow head
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowheadWidth/2);
            mergeContext.lineTo(currX + arrowLength/2, arrowPosY);
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowheadWidth/2);
            mergeContext.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowWidth/2);

            // Go back to the left until meeting left arrow head
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowWidth/2);
            
            // Draw left arrow head
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowheadWidth/2);
            mergeContext.lineTo(currX - arrowLength/2, arrowPosY);
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY  - arrowheadWidth/2);
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY);
            
            mergeContext.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY - arrowWidth/2);
            mergeContext.lineTo(currX, arrowPosY - arrowWidth/2);

            mergeContext.closePath();

            mergeContext.fillStyle = "#444444";
            mergeContext.fill();



            /////////////////////////////////////////////////////

            mergeContext2.arc(currX, arrowPosY, arrowLength*0.7, 0, Math.PI * 2, false);
            mergeContext2.fillStyle = "#FFD79340";
            mergeContext2.fill()


            // Draw border
            mergeContext2.beginPath();
            mergeContext2.moveTo(vidWidth*position, 0);
            mergeContext2.lineTo(vidWidth*position, vidHeight);
            mergeContext2.closePath()
            mergeContext2.strokeStyle = "#444444";
            mergeContext2.lineWidth = 5;            
            mergeContext2.stroke();

            // Draw arrow
            mergeContext2.beginPath();
            mergeContext2.moveTo(currX, arrowPosY - arrowWidth/2);
            
            // Move right until meeting arrow head
            mergeContext2.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowWidth/2);
            
            // Draw right arrow head
            mergeContext2.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowheadWidth/2);
            mergeContext2.lineTo(currX + arrowLength/2, arrowPosY);
            mergeContext2.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowheadWidth/2);
            mergeContext2.lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowWidth/2);

            // Go back to the left until meeting left arrow head
            mergeContext2.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowWidth/2);
            
            // Draw left arrow head
            mergeContext2.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowheadWidth/2);
            mergeContext2.lineTo(currX - arrowLength/2, arrowPosY);
            mergeContext2.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY  - arrowheadWidth/2);
            mergeContext2.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY);
            
            mergeContext2.lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY - arrowWidth/2);
            mergeContext2.lineTo(currX, arrowPosY - arrowWidth/2);

            mergeContext2.closePath();

            mergeContext2.fillStyle = "#444444";
            mergeContext2.fill();

            
            
        }
        requestAnimationFrame(drawLoop);
    } 
}

Number.prototype.clamp = function(min, max) {
  return Math.min(Math.max(this, min), max);
};
    
    
function resizeAndPlayDual(element, id2)
{
  var cv = document.getElementById(element.id + "Merge");
  cv.width = element.videoWidth/2;
  cv.height = element.videoHeight;

  console.log("video height:" + element.videoHeight)
  console.log("video width:" + element.videoWidth)

  element.play();
  element.style.height = "0px";  // Hide video without stopping it

  var element2 = document.getElementById(id2);
  var cv2 = document.getElementById(element2.id + "Merge");
  cv2.width = element2.videoWidth/2;
  cv2.height = element2.videoHeight;

  console.log("video height:" + element2.videoHeight)
  console.log("video width:" + element2.videoWidth)

  element2.play();
  element2.style.height = "0px";  // Hide video without stopping it
    
  // playVidsDual(element.id, element2.id);
  playVidsMulti([element.id, element2.id]);
}


function resizeAndPlayDualWhenReady(element, id2) {
    var element2 = document.getElementById(id2);
    var cnt = 0;
    setTimeout(function playIfReady() {
        if ((element2.readyState != 4) || (element.readyState != 4)){
            console.log("second video is not ready yet, cnt=" + cnt);
            cnt++;
            // Bail out if it retries for more than 10 seconds.
            if (cnt < 1000) {
                setTimeout(playIfReady, 10);
            }
        }
        resizeAndPlayDual(element, id2);
    }, 10);
}


// version of multiple videos

function playVidsMulti(videoIds) {
    var video_num = videoIds.length;
    if (video_num === 0) {
        console.log("videoIds is empty. ")
        return;
    }
    var videoMerges = [];
    var vids = [];
    var mergeContexts = [];

    for (var i = 0; i < video_num; i++) {
        videoMerges.push(document.getElementById(videoIds[i] + "Merge"));
        vids.push(document.getElementById(videoIds[i]));
        mergeContexts.push(videoMerges[i].getContext("2d"));
    }

    var position = 0.5;
    var vidWidth = vids[0].videoWidth/2;
    var vidHeight = vids[0].videoHeight;

    if (vids.every(vid => vid.readyState > 3)) {
        for (var i =0; i < video_num; i++) {
            vids[i].play();
        }
        function trackLocation(e) {
            // Normalize to [0, 1]
            for (var idx = 0; idx < video_num; idx++) {
                bcr = videoMerges[idx].getBoundingClientRect();
                position = ((e.pageX - bcr.x) / bcr.width);
                if (position < 0) {
                    position = (idx===0) ? 0 : 1;
                    break;
                } else if (position < 1) {
                    break;
                }
            }
            if (position > 1){
                position = 1;
            }
            // stop other video
        }
        function trackLocationTouch(e) {
            // Normalize to [0, 1]
            for (var idx = 0; idx < video_num; idx++) {
                bcr = videoMerges[idx].getBoundingClientRect();
                position = ((e.touches[0].pageX - bcr.x) / bcr.width);
                if (position < 0) {
                    position = (idx===0) ? 0 : 1;
                    break;
                } else if (position < 1) {
                    break;
                }
            }
            if (position > 1){
                position = 1;
            }
        }
        for (var i = 0; i < video_num; i++) {
            videoMerges[i].addEventListener("mousemove",  trackLocation, false); 
            videoMerges[i].addEventListener("touchstart", trackLocationTouch, false);
            videoMerges[i].addEventListener("touchmove",  trackLocationTouch, false);
        }
        function drawLoop() {
            var colStart = (vidWidth * position).clamp(0.0, vidWidth);
            var colWidth = (vidWidth - (vidWidth * position)).clamp(0.0, vidWidth);
            for (var i = 0; i < video_num; i++) {
                mergeContexts[i].drawImage(vids[i], 0, 0, vidWidth, vidHeight, 0, 0, vidWidth, vidHeight);
                mergeContexts[i].drawImage(vids[i], colStart+vidWidth, 0, colWidth, vidHeight, colStart, 0, colWidth, vidHeight);
            }
            requestAnimationFrame(drawLoop);

            var arrowLength = 0.09 * vidHeight;
            var arrowheadWidth = 0.025 * vidHeight;
            var arrowheadLength = 0.04 * vidHeight;
            var arrowPosY = vidHeight / 10;
            var arrowWidth = 0.007 * vidHeight;
            var currX = vidWidth * position;

            for (var i = 0; i < video_num; i++) {
                // Draw circle
                mergeContexts[i].arc(currX, arrowPosY, arrowLength*0.7, 0, Math.PI * 2, false);
                mergeContexts[i].fillStyle = "#FFD79340";
                mergeContexts[i].fill();

                // Draw border
                mergeContexts[i].beginPath();
                mergeContexts[i].moveTo(vidWidth*position, 0);
                mergeContexts[i].lineTo(vidWidth*position, vidHeight);
                mergeContexts[i].closePath()
                mergeContexts[i].strokeStyle = "#444444";
                mergeContexts[i].lineWidth = 5;            
                mergeContexts[i].stroke();

                // Draw arrow
                mergeContexts[i].beginPath();
                mergeContexts[i].moveTo(currX, arrowPosY - arrowWidth/2);

                // Move right until meeting arrow head
                mergeContexts[i].lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowWidth/2);
            
                // Draw right arrow head
                mergeContexts[i].lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY - arrowheadWidth/2);
                mergeContexts[i].lineTo(currX + arrowLength/2, arrowPosY);
                mergeContexts[i].lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowheadWidth/2);
                mergeContexts[i].lineTo(currX + arrowLength/2 - arrowheadLength/2, arrowPosY + arrowWidth/2);

                // Go back to the left until meeting left arrow head
                mergeContexts[i].lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowWidth/2);
            
                // Draw left arrow head
                mergeContexts[i].lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY + arrowheadWidth/2);
                mergeContexts[i].lineTo(currX - arrowLength/2, arrowPosY);
                mergeContexts[i].lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY  - arrowheadWidth/2);
                mergeContexts[i].lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY);
            
                mergeContexts[i].lineTo(currX - arrowLength/2 + arrowheadLength/2, arrowPosY - arrowWidth/2);
                mergeContexts[i].lineTo(currX, arrowPosY - arrowWidth/2);

                mergeContexts[i].closePath();

                mergeContexts[i].fillStyle = "#444444";
                mergeContexts[i].fill();
            }
        }
        requestAnimationFrame(drawLoop);
    }

}

function resizeAndPlayMulti(ids)
{
  var elements = [];
  var cvs = [];
  for (var i = 0; i < ids.length; i++) {
    elements.push(document.getElementById(ids[i]));
    cvs.push(document.getElementById(elements[i].id + "Merge"));
    cvs[i].width = elements[i].videoWidth / 2;
    cvs[i].height = elements[i].videoHeight;

    console.log("video height:" + elements[i].videoHeight);
    console.log("video width:" + elements[i].videoWidth);

    elements[i].style.height = "0px"; // Hide video without stopping it
  }

  playVidsMulti(ids);
}

function resizeAndPlayMultiWhenReady(element, ids) {
    var elements = [element];
    for (var i = 0; i < ids.length; i++) {
        elements.push(document.getElementById(ids[i]));
    }
    var new_ids = [element.id, ...ids];
    var cnt = 0;
    setTimeout(function playIfReady() {
        if (elements.some(element => element.readyState != 4)) {
            console.log("These videos are not ready yet, cnt=" + cnt);
            cnt++;
            // Bail out if it retries for more than 10 seconds.
            if (cnt < 1000) {
                setTimeout(playIfReady, 10);
            }
        }
        resizeAndPlayMulti(new_ids);
    }, 10);
}