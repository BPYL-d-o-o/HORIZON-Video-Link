    .section .rodata.video_image, "a"
    .balign 4
    .global video_image
    .global video_image_end
video_image:
    .incbin VIDEO_IMAGE_PATH
video_image_end:
